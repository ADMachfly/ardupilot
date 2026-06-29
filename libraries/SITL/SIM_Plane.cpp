/*
   This program is free software: you can redistribute it and/or modify
   it under the terms of the GNU General Public License as published by
   the Free Software Foundation, either version 3 of the License, or
   (at your option) any later version.

   This program is distributed in the hope that it will be useful,
   but WITHOUT ANY WARRANTY; without even the implied warranty of
   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
   GNU General Public License for more details.

   You should have received a copy of the GNU General Public License
   along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */
/*
  very simple plane simulator class. Not aerodynamically accurate,
  just enough to be able to debug control logic for new frame types
*/

#include "SIM_Plane.h"

#include <stdio.h>
#include <AP_Filesystem/AP_Filesystem_config.h>
#include <AP_Filesystem/AP_Filesystem.h>

using namespace SITL;

// ---------------------------------------------------------------------------
// SR-75 fuel state (file-scope, SITL-only)
// ---------------------------------------------------------------------------
static constexpr float SR75_FUEL_CAPACITY_ML = 30000.0f;  // 30 L
static constexpr float SR75_IDLE_THRUST_N    = 30.0f;
static constexpr float SR75_MAX_THRUST_N     = 800.0f;
static constexpr float SR75_IDLE_FLOW_ML_MIN = 200.0f;
static constexpr float SR75_MAX_FLOW_ML_MIN  = 1392.0f;

static float sr75_fuel_ml         = SR75_FUEL_CAPACITY_ML;
static float sr75_fuel_flow_mlmin = 0.0f;
static bool  sr75_fuel_empty      = false;

float Plane::get_sr75_fuel_ml()
{
    return sr75_fuel_ml;
}

float Plane::get_sr75_fuel_flow_mlmin()
{
    return sr75_fuel_flow_mlmin;
}

bool Plane::get_sr75_fuel_empty()
{
    return sr75_fuel_empty;
}

Plane::Plane(const char *frame_str) :
    Aircraft(frame_str)
{

    const char *colon = strchr(frame_str, ':');
    size_t slen = strlen(frame_str);
    // The last 5 letters are ".json"
    if (colon != nullptr && slen > 5 && strcmp(&frame_str[slen-5], ".json") == 0) {
        load_coeffs(colon+1);
    } else {
        coefficient = default_coefficients;
    }

    mass = 2.0f;

    /*
       scaling from motor power to Newtons. Allows the plane to hold
       vertically against gravity when the motor is at hover_throttle
    */
    thrust_scale = (mass * GRAVITY_MSS) / hover_throttle;
    frame_height = 0.1f;

    ground_behavior = GROUND_BEHAVIOR_FWD_ONLY;
    lock_step_scheduled = true;

    if (strstr(frame_str, "-heavy")) {
        mass = 8;
    }
    if (strstr(frame_str, "-jet")) {
        // a 22kg "jet", level top speed is 102m/s
        mass = 22;
        thrust_scale = (mass * GRAVITY_MSS) / hover_throttle;
    }
    if (strstr(frame_str, "-sr75")) {
        // SR-75 jet powered UAV - 82.5kg, 800N thrust, 125 m/s max
        mass = 82.5;
        thrust_scale = 800.0f;
        is_sr75 = true;

        coefficient.c_drag_p = 0.04;
        coefficient.c_drag_deltae = 0.08;
        coefficient.c_m_deltae = 1.2;
    }
    if (strstr(frame_str, "-revthrust")) {
        reverse_thrust = true;
    }
    if (strstr(frame_str, "-elevon")) {
        elevons = true;
    } else if (strstr(frame_str, "-vtail")) {
        vtail = true;
    } else if (strstr(frame_str, "-dspoilers")) {
        dspoilers = true;
    } else if (strstr(frame_str, "-redundant")) {
        redundant = true;
    }
    if (strstr(frame_str, "-elevrev")) {
        reverse_elevator_rudder = true;
    }
    if (strstr(frame_str, "-catapult")) {
        have_launcher = true;
        launch_accel = 15;
        launch_time = 2;
    }
    if (strstr(frame_str, "-bungee")) {
        have_launcher = true;
        launch_accel = 7;
        launch_time = 4;
    }
    if (strstr(frame_str, "-throw")) {
        have_launcher = true;
        launch_accel = 25;
        launch_time = 0.4;
    }
    if (strstr(frame_str, "-tailsitter")) {
        tailsitter = true;
        ground_behavior = GROUND_BEHAVIOR_TAILSITTER;
        thrust_scale *= 1.5;
    }
    if (strstr(frame_str, "-steering")) {
        have_steering = true;
    }

#if AP_FILESYSTEM_FILE_READING_ENABLED
    if (strstr(frame_str, "-3d")) {
        aerobatic = true;
        thrust_scale *= 1.5;
        // setup parameters for plane-3d
        AP_Param::load_defaults_file("@ROMFS/models/plane.parm", false);
        AP_Param::load_defaults_file("@ROMFS/models/plane-3d.parm", false);
    }
#endif

    if (strstr(frame_str, "-ice")) {
        ice_engine = true;
    }

    if (strstr(frame_str, "-soaring")) {
        mass = 2.0;
        coefficient.c_drag_p = 0.05;
    }
}

void Plane::load_coeffs(const char *model_json)
{
    char *fname = nullptr;
    struct stat st;

    if (AP::FS().stat(model_json, &st) == 0) {
        fname = strdup(model_json);
    } else {
        IGNORE_RETURN(asprintf(&fname, "@ROMFS/models/%s", model_json));
        if (fname == nullptr || AP::FS().stat(fname, &st) != 0) {
            AP_HAL::panic("%s failed to load", model_json);
        }
    }

    AP_JSON::value *obj = AP_JSON::load_json(fname);
    if (obj == nullptr) {
        AP_HAL::panic("%s failed to load", fname);
    }

    enum class VarType {
        FLOAT,
        VECTOR3F,
    };

    struct json_search {
        const char *label;
        void *ptr;
        VarType t;
    };
    
    json_search vars[] = {
#define COFF_FLOAT(s) { #s, &coefficient.s, VarType::FLOAT }
        COFF_FLOAT(s),
        COFF_FLOAT(b),
        COFF_FLOAT(c),
        COFF_FLOAT(c_lift_0),
        COFF_FLOAT(c_lift_deltae),
        COFF_FLOAT(c_lift_a),
        COFF_FLOAT(c_lift_q),
        COFF_FLOAT(mcoeff),
        COFF_FLOAT(oswald),
        COFF_FLOAT(alpha_stall),
        COFF_FLOAT(c_drag_q),
        COFF_FLOAT(c_drag_deltae),
        COFF_FLOAT(c_drag_p),
        COFF_FLOAT(c_y_0),
        COFF_FLOAT(c_y_b),
        COFF_FLOAT(c_y_p),
        COFF_FLOAT(c_y_r),
        COFF_FLOAT(c_y_deltaa),
        COFF_FLOAT(c_y_deltar),
        COFF_FLOAT(c_l_0),
        COFF_FLOAT(c_l_p),
        COFF_FLOAT(c_l_b),
        COFF_FLOAT(c_l_r),
        COFF_FLOAT(c_l_deltaa),
        COFF_FLOAT(c_l_deltar),
        COFF_FLOAT(c_m_0),
        COFF_FLOAT(c_m_a),
        COFF_FLOAT(c_m_q),
        COFF_FLOAT(c_m_deltae),
        COFF_FLOAT(c_n_0),
        COFF_FLOAT(c_n_b),
        COFF_FLOAT(c_n_p),
        COFF_FLOAT(c_n_r),
        COFF_FLOAT(c_n_deltaa),
        COFF_FLOAT(c_n_deltar),
        COFF_FLOAT(deltaa_max),
        COFF_FLOAT(deltae_max),
        COFF_FLOAT(deltar_max),
        { "CGOffset", &coefficient.CGOffset, VarType::VECTOR3F },
    };

    for (uint8_t i=0; i<ARRAY_SIZE(vars); i++) {
        auto v = obj->get(vars[i].label);
        if (v.is<AP_JSON::null>()) {
            // use default value
            continue;
        }
        if (vars[i].t == VarType::FLOAT) {
            parse_float(v, vars[i].label, *((float *)vars[i].ptr));

        } else if (vars[i].t == VarType::VECTOR3F) {
            parse_vector3(v, vars[i].label, *(Vector3f *)vars[i].ptr);

        }
    }

    delete obj;

    ::printf("Loaded plane aero coefficients from %s\n", model_json);
}

void Plane::parse_float(AP_JSON::value val, const char* label, float &param) {
    if (!val.is<double>()) {
        AP_HAL::panic("Bad json type for %s: %s", label, val.to_str().c_str());
    }
    param = val.get<double>();
}

void Plane::parse_vector3(AP_JSON::value val, const char* label, Vector3f &param) {
    if (!val.is<AP_JSON::value::array>() || !val.contains(2) || val.contains(3)) {
        AP_HAL::panic("Bad json type for %s: %s", label, val.to_str().c_str());
    }
    for (uint8_t j=0; j<3; j++) {
        parse_float(val.get(j), label, param[j]);
    }
}

/*
  the following functions are from last_letter
  https://github.com/Georacer/last_letter/blob/master/last_letter/src/aerodynamicsLib.cpp
  many thanks to Georacer!
 */
float Plane::liftCoeff(float alpha) const
{
    const float alpha0 = coefficient.alpha_stall;
    const float M = coefficient.mcoeff;
    const float c_lift_0 = coefficient.c_lift_0;
    const float c_lift_a0 = coefficient.c_lift_a;

    // clamp the value of alpha to avoid exp(90) in calculation of sigmoid
    const float max_alpha_delta = 0.8f;
    if (alpha-alpha0 > max_alpha_delta) {
        alpha = alpha0 + max_alpha_delta;
    } else if (alpha0-alpha > max_alpha_delta) {
        alpha = alpha0 - max_alpha_delta;
    }
	double sigmoid = ( 1+exp(-M*(alpha-alpha0))+exp(M*(alpha+alpha0)) ) / (1+exp(-M*(alpha-alpha0))) / (1+exp(M*(alpha+alpha0)));
	double linear = (1.0-sigmoid) * (c_lift_0 + c_lift_a0*alpha); //Lift at small AoA
	double flatPlate = sigmoid*(2*copysign(1,alpha)*pow(sin(alpha),2)*cos(alpha)); //Lift beyond stall

	float result  = linear+flatPlate;
	return result;
}

float Plane::dragCoeff(float alpha) const
{
    const float b = coefficient.b;
    const float s = coefficient.s;
    const float c_drag_p = coefficient.c_drag_p;
    const float c_lift_0 = coefficient.c_lift_0;
    const float c_lift_a0 = coefficient.c_lift_a;
    const float oswald = coefficient.oswald;

    const double AR = pow(b, 2) / s;
    const double cl = c_lift_0 + c_lift_a0 * alpha;

    double effective_cd_p = c_drag_p;
    double induced_denom  = M_PI * oswald * AR;
    double wave_drag      = 0.0;

    if (is_sr75 && airspeed > 10.0f) {
        // ---------------------------------------------------------------
        // 1. Reynolds-number correction to parasitic drag (CD_p)
        //
        // Turbulent skin friction: Cf ∝ Re^(-0.2)
        // Re = ρ·V·c / μ,  with μ ∝ ρ^0.176 (Sutherland, ISA troposphere)
        // Combined ratio from any (ρ,V) to the reference (ρ₀=1.225, V₀=69):
        //   CD_p_factor = (ρ₀/ρ)^0.165 × (V₀/V)^0.2
        //
        // Effect:
        //   +5000 m, same TAS  → factor ≈ 1.087  (+8.7% parasitic drag)
        //   SL,      125 m/s   → factor ≈ 0.884  (-11.6% parasitic drag)
        //   +5000 m, 125 m/s   → factor ≈ 0.960  (near neutral — Re effects cancel)
        // ---------------------------------------------------------------
        const float re_factor = powf(1.225f / air_density, 0.1648f) *
                                powf(69.0f  / airspeed,    0.2f);
        effective_cd_p = c_drag_p * (double)re_factor;

        // ---------------------------------------------------------------
        // 2. Mach-number corrections
        //
        // Speed of sound corrected for altitude (ISA troposphere):
        //   a = a₀ × (ρ/ρ₀)^0.1175
        // ---------------------------------------------------------------
        const float sos  = 340.3f * powf(air_density / 1.225f, 0.1175f);
        const float mach = airspeed / sos;
        const float m2   = mach * mach;

        if (m2 < 0.98f) {
            // Prandtl-Glauert: induced drag rises by 1/(1-M²) at same alpha.
            // Weight reduction lowers alpha via autopilot trim → lower CL
            // → less induced drag, captured here automatically.
            induced_denom *= (1.0 - m2);
        }

        // Wave drag onset above M=0.5
        if (mach > 0.5f) {
            const double dm = mach - 0.5;
            wave_drag = 0.005 * dm * dm;
        }
    }

    return (float)(effective_cd_p + wave_drag + cl * cl / induced_denom);
}

// Torque calculation function
Vector3f Plane::getTorque(float inputAileron, float inputElevator, float inputRudder, float inputThrust, const Vector3f &force) const
{
    float alpha = angle_of_attack;

	//calculate aerodynamic torque
    float effective_airspeed = airspeed;

    if (tailsitter || aerobatic) {
        /*
          tailsitters get airspeed from prop-wash
         */
        effective_airspeed += inputThrust * 20;

        // reduce effective angle of attack as thrust increases
        alpha *= constrain_float(1 - inputThrust, 0, 1);
    }
    
    const float s = coefficient.s;
    const float c = coefficient.c;
    const float b = coefficient.b;
    const float c_l_0 = coefficient.c_l_0;
    const float c_l_b = coefficient.c_l_b;
    const float c_l_p = coefficient.c_l_p;
    const float c_l_r = coefficient.c_l_r;
    const float c_l_deltaa = coefficient.c_l_deltaa;
    const float c_l_deltar = coefficient.c_l_deltar;
    const float c_m_0 = coefficient.c_m_0;
    const float c_m_a = coefficient.c_m_a;
    const float c_m_q = coefficient.c_m_q;
    const float c_m_deltae = coefficient.c_m_deltae;
    const float c_n_0 = coefficient.c_n_0;
    const float c_n_b = coefficient.c_n_b;
    const float c_n_p = coefficient.c_n_p;
    const float c_n_r = coefficient.c_n_r;
    const float c_n_deltaa = coefficient.c_n_deltaa;
    const float c_n_deltar = coefficient.c_n_deltar;
    const Vector3f &CGOffset = coefficient.CGOffset;
    
    float rho = air_density;

	//read angular rates
	double p = gyro.x;
	double q = gyro.y;
	double r = gyro.z;

	double qbar = 1.0/2.0*rho*pow(effective_airspeed,2)*s; //Calculate dynamic pressure
	double la, na, ma;
	if (is_zero(effective_airspeed))
	{
		la = 0;
		ma = 0;
		na = 0;
	}
	else
	{
		la = qbar*b*(c_l_0 + c_l_b*beta + c_l_p*b*p/(2*effective_airspeed) + c_l_r*b*r/(2*effective_airspeed) + c_l_deltaa*inputAileron + c_l_deltar*inputRudder);
		ma = qbar*c*(c_m_0 + c_m_a*alpha + c_m_q*c*q/(2*effective_airspeed) + c_m_deltae*inputElevator);
		na = qbar*b*(c_n_0 + c_n_b*beta + c_n_p*b*p/(2*effective_airspeed) + c_n_r*b*r/(2*effective_airspeed) + c_n_deltaa*inputAileron + c_n_deltar*inputRudder);
	}


	// Add torque to force misalignment with CG
	// r x F, where r is the distance from CoG to CoL
	la +=  CGOffset.y * force.z - CGOffset.z * force.y;
	ma += -CGOffset.x * force.z + CGOffset.z * force.x;
	na += -CGOffset.y * force.x + CGOffset.x * force.y;

	return Vector3f(la, ma, na);
}

// Force calculation function from last_letter
Vector3f Plane::getForce(float inputAileron, float inputElevator, float inputRudder) const
{
    const float alpha = angle_of_attack;
    const float c_drag_q = coefficient.c_drag_q;
    const float c_lift_q = coefficient.c_lift_q;
    const float s = coefficient.s;
    const float c = coefficient.c;
    const float b = coefficient.b;
    const float c_drag_deltae = coefficient.c_drag_deltae;
    const float c_lift_deltae = coefficient.c_lift_deltae;
    const float c_y_0 = coefficient.c_y_0;
    const float c_y_b = coefficient.c_y_b;
    const float c_y_p = coefficient.c_y_p;
    const float c_y_r = coefficient.c_y_r;
    const float c_y_deltaa = coefficient.c_y_deltaa;
    const float c_y_deltar = coefficient.c_y_deltar;
    
    float rho = air_density;

	//request lift and drag alpha-coefficients from the corresponding functions
	double c_lift_a = liftCoeff(alpha);
	double c_drag_a = dragCoeff(alpha);

	//convert coefficients to the body frame
	double c_x_a = -c_drag_a*cos(alpha)+c_lift_a*sin(alpha);
	double c_x_q = -c_drag_q*cos(alpha)+c_lift_q*sin(alpha);
	double c_z_a = -c_drag_a*sin(alpha)-c_lift_a*cos(alpha);
	double c_z_q = -c_drag_q*sin(alpha)-c_lift_q*cos(alpha);

	//read angular rates
	double p = gyro.x;
	double q = gyro.y;
	double r = gyro.z;

	//calculate aerodynamic force
	double qbar = 1.0/2.0*rho*pow(airspeed,2)*s; //Calculate dynamic pressure
	double ax, ay, az;
	if (is_zero(airspeed))
	{
		ax = 0;
		ay = 0;
		az = 0;
	}
	else
	{
		ax = qbar*(c_x_a + c_x_q*c*q/(2*airspeed) - c_drag_deltae*cos(alpha)*fabs(inputElevator) + c_lift_deltae*sin(alpha)*inputElevator);
		// split c_x_deltae to include "abs" term
		ay = qbar*(c_y_0 + c_y_b*beta + c_y_p*b*p/(2*airspeed) + c_y_r*b*r/(2*airspeed) + c_y_deltaa*inputAileron + c_y_deltar*inputRudder);
		az = qbar*(c_z_a + c_z_q*c*q/(2*airspeed) - c_drag_deltae*sin(alpha)*fabs(inputElevator) - c_lift_deltae*cos(alpha)*inputElevator);
		// split c_z_deltae to include "abs" term
	}
    return Vector3f(ax, ay, az);
}

void Plane::calculate_forces(const struct sitl_input &input, Vector3f &rot_accel)
{
    float aileron  = filtered_servo_angle(input, 0);
    float elevator = filtered_servo_angle(input, 1);
    float rudder   = filtered_servo_angle(input, 3);
    bool launch_triggered = input.servos[6] > 1700;
    if (reverse_elevator_rudder) {
        elevator = -elevator;
        rudder = -rudder;
    }
    if (elevons) {
        // fake an elevon plane
        float ch1 = aileron;
        float ch2 = elevator;
        aileron  = (ch2-ch1)/2.0f;
        // the minus does away with the need for RC2_REVERSED=-1
        elevator = -(ch2+ch1)/2.0f;

        // assume no rudder
        rudder = 0;
    } else if (vtail) {
        // fake a vtail plane
        float ch1 = elevator;
        float ch2 = rudder;
        // this matches VTAIL_OUTPUT==2
        elevator = (ch2-ch1)/2.0f;
        rudder   = (ch2+ch1)/2.0f;
    } else if (dspoilers) {
        // fake a differential spoiler plane. Use outputs 1, 2, 4 and 5
        float dspoiler1_left = filtered_servo_angle(input, 0);
        float dspoiler1_right = filtered_servo_angle(input, 1);
        float dspoiler2_left = filtered_servo_angle(input, 3);
        float dspoiler2_right = filtered_servo_angle(input, 4);
        float elevon_left  = (dspoiler1_left + dspoiler2_left)/2;
        float elevon_right = (dspoiler1_right + dspoiler2_right)/2;
        aileron  = (elevon_right-elevon_left)/2;
        elevator = (elevon_left+elevon_right)/2;
        rudder = fabsf(dspoiler1_right - dspoiler2_right)/2 - fabsf(dspoiler1_left - dspoiler2_left)/2;
    } else if (redundant) {
        // channels 1/9 are left/right ailierons
        // channels 2/10 are left/right elevators
        // channels 4/12 are top/bottom rudders
        aileron  = (filtered_servo_angle(input, 0) + filtered_servo_angle(input, 8)) / 2.0;
        elevator = (filtered_servo_angle(input, 1) + filtered_servo_angle(input, 9)) / 2.0;
        rudder   = (filtered_servo_angle(input, 3) + filtered_servo_angle(input, 11)) / 2.0;
    }
    //printf("Aileron: %.1f elevator: %.1f rudder: %.1f\n", aileron, elevator, rudder);

    float thrust = reverse_thrust ? filtered_servo_angle(input, 2) : filtered_servo_range(input, 2);

    if (ice_engine) {
        thrust = icengine.update(input);
    }

    // calculate angle of attack
    angle_of_attack = atan2f(velocity_air_bf.z, velocity_air_bf.x);
    beta = atan2f(velocity_air_bf.y,velocity_air_bf.x);

    if (tailsitter || aerobatic) {
        /*
          tailsitters get 4x the control surfaces
         */
        aileron *= 4;
        elevator *= 4;
        rudder *= 4;
    }
    
    Vector3f force = getForce(aileron, elevator, rudder);
    rot_accel = getTorque(aileron, elevator, rudder, thrust, force);

    if (have_launcher) {
        /*
          simple simulation of a launcher
         */
        if (launch_triggered) {
            uint64_t now = AP_HAL::millis64();
            if (launch_start_ms == 0) {
                launch_start_ms = now;
            }
            if (now - launch_start_ms < launch_time*1000) {
                force.x += mass * launch_accel;
                force.z += mass * launch_accel/3;
            }
        } else {
            // allow reset of catapult
            launch_start_ms = 0;
        }
    }
    
    // simulate engine RPM
    motor_mask |= (1U<<2);
    rpm[2] = thrust * 7000;
    
// scale normal engine thrust to Newtons
thrust *= thrust_scale;

// ---------------------------------------------------------------------------
// SR-75 fuel consumption model
// ---------------------------------------------------------------------------
if (is_sr75) {
    static uint32_t sr75_fuel_last_ms = 0;
    const uint32_t now_ms = AP_HAL::millis();

    if (sr75_fuel_last_ms == 0) {
        sr75_fuel_last_ms = now_ms;
    }

    const float dt_min = (now_ms - sr75_fuel_last_ms) * (1.0f / 60000.0f);
    sr75_fuel_last_ms = now_ms;

    if (!sr75_fuel_empty && dt_min > 0.0f) {
        const float thrust_n = thrust;  // already in Newtons
        if (thrust_n <= 0.0f) {
            sr75_fuel_flow_mlmin = 0.0f;
        } else if (thrust_n < SR75_IDLE_THRUST_N) {
            sr75_fuel_flow_mlmin = (thrust_n / SR75_IDLE_THRUST_N) * SR75_IDLE_FLOW_ML_MIN;
        } else {
            sr75_fuel_flow_mlmin = SR75_IDLE_FLOW_ML_MIN +
                (thrust_n - SR75_IDLE_THRUST_N) *
                ((SR75_MAX_FLOW_ML_MIN - SR75_IDLE_FLOW_ML_MIN) /
                 (SR75_MAX_THRUST_N   - SR75_IDLE_THRUST_N));
        }

        sr75_fuel_ml -= sr75_fuel_flow_mlmin * dt_min;
        if (sr75_fuel_ml <= 0.0f) {
            sr75_fuel_ml   = 0.0f;
            sr75_fuel_empty = true;
            ::printf("SR75: FUEL EXHAUSTED — engine flameout\n");
        }

        // Console log every ~10 s
        static uint32_t sr75_fuel_log_ms = 0;
        if (now_ms - sr75_fuel_log_ms >= 10000) {
            sr75_fuel_log_ms = now_ms;
            const float sos  = 340.3f * powf(air_density / 1.225f, 0.1175f);
            const float mach = airspeed / sos;
            const float cd   = dragCoeff(angle_of_attack);
            ::printf("SR75: fuel=%.0f ml  flow=%.1f ml/min  thr=%.1f N\n"
                     "      M=%.3f  rho=%.3f kg/m3  CD=%.4f\n",
                     (double)sr75_fuel_ml, (double)sr75_fuel_flow_mlmin, (double)thrust_n,
                     (double)mach, (double)air_density, (double)cd);
        }
    }

    if (sr75_fuel_empty) {
        thrust = 0.0f;
        sr75_fuel_flow_mlmin = 0.0f;
    }
}

// Total force in body frame before mass division.
// Existing model already has:
//   thrust = engine force along body X
//   force  = aerodynamic/launch forces
Vector3f total_force = Vector3f(thrust, 0, 0) + force;

// -----------------------------------------------------------------------------
// SR-75 temporary RATO SITL physics model
//
// This is a simulation-only booster force model.
// It does not yet read RATO_ENABLE from ArduPlane parameters.
// It is intentionally temporary for Step 6.
//
// Current model:
//   RATO thrust = 5800 N
//   RATO mass   = 10 kg
//   burn time   = 3 seconds
//   pitch angle = 20 deg
//
// During burn:
//   total aircraft mass = aircraft mass + RATO mass
//   force is added in body X/Z direction
//
// After burn:
//   RATO thrust becomes zero
//   booster mass is still attached in this step
//   ejection/mass drop will be added in the next step
// -----------------------------------------------------------------------------

static bool sr75_rato_started  = false;
static bool sr75_rato_burning  = false;
static bool sr75_rato_attached = false;
static uint32_t sr75_rato_start_ms = 0;

const float sr75_rato_thrust_N  = 5800.0f;
const float sr75_rato_mass_kg   = 10.0f;
const float sr75_rato_burn_s    = 3.0f;
const float sr75_rato_pitch_deg = 20.0f;

// TEMP SITL-only autostart.
// Later connect this to ArduPlane RATOController state.
if (!sr75_rato_started && thrust > 0.9f * thrust_scale && position.z > -2.0f) {
    sr75_rato_started  = true;
    sr75_rato_burning  = true;
    sr75_rato_attached = true;
    sr75_rato_start_ms = AP_HAL::millis();
}

float effective_mass = mass;

// Booster mass applies only while booster is attached.
if (sr75_rato_attached) {
    effective_mass += sr75_rato_mass_kg;
}

// SR-75: subtract burned fuel mass (30 L full load = 24 kg at 0.8 kg/L).
// mass=82.5 kg is the full-fuel weight; dry weight is ~58.5 kg.
if (is_sr75) {
    const float fuel_burned_kg = (SR75_FUEL_CAPACITY_ML - sr75_fuel_ml) * 0.0008f;
    effective_mass -= fuel_burned_kg;
}

if (sr75_rato_burning) {
    const float rato_time_s = (AP_HAL::millis() - sr75_rato_start_ms) * 0.001f;

    if (rato_time_s <= sr75_rato_burn_s) {
        const float rato_pitch_rad = radians(sr75_rato_pitch_deg);

        total_force.x += sr75_rato_thrust_N * cosf(rato_pitch_rad);
        total_force.z -= sr75_rato_thrust_N * sinf(rato_pitch_rad);
    } else {
        sr75_rato_burning  = false;
        sr75_rato_attached = false;  // eject booster at burnout
    }
}
// Convert total body-frame force to body-frame acceleration.
accel_body = total_force;
accel_body /= effective_mass;

    // add some noise
    if (thrust_scale > 0) {
        add_noise(fabsf(thrust) / thrust_scale);
    }

    if (on_ground() && !tailsitter) {
        // add some ground friction
        Vector3f vel_body = dcm.transposed() * velocity_ef;
        accel_body.x -= vel_body.x * 0.3f;
    }
}
    
/*
  update the plane simulation by one time step
 */
void Plane::update(const struct sitl_input &input)
{
    Vector3f rot_accel;

    update_wind(input);
    
    calculate_forces(input, rot_accel);

    float throttle = reverse_thrust ? filtered_servo_angle(input, 2) : filtered_servo_range(input, 2);
    if (is_sr75) {
        // Expose SR-75 fuel state via battery telemetry:
        //   voltage  → fuel level (0–batt_voltage mapped to 0–15 L)
        //   current  → fuel flow rate in ml/min
        battery_voltage = sitl->batt_voltage * (sr75_fuel_ml / SR75_FUEL_CAPACITY_ML);
        battery_current = sr75_fuel_flow_mlmin;
    } else {
        battery_voltage = sitl->batt_voltage - 0.7*throttle;
        battery_current = (battery_voltage/sitl->batt_voltage)*50.0f*sq(throttle);
    }

    update_dynamics(rot_accel);

    /*
      add in ground steering, this should be replaced with a proper
      calculation of a nose wheel effect
    */
    if (have_steering && on_ground()) {
        const float steering = filtered_servo_angle(input, 4);
        const Vector3f velocity_bf = dcm.transposed() * velocity_ef;
        const float steer_scale = radians(5);
        gyro.z += steering * velocity_bf.x * steer_scale;
    }

    update_external_payload(input);

    // update lat/lon/altitude
    update_position();
    time_advance();

    // update magnetic field
    update_mag_field_bf();
}

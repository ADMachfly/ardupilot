# SR-75 Layer 2 External JSBSim HIL Bridge

Purpose:
Connect JSBSim SR-75 6DOF simulation to a real Pixhawk 2.4.8 running custom SR-75 ArduPlane firmware.

Architecture:
JSBSim 6DOF synthetic state -> MAVLink HIL/GPS/sensor messages -> Pixhawk
Pixhawk SERVO_OUTPUT_RAW -> bridge -> JSBSim control inputs

Safety:
No live engine, no live fuel pump, no live RATO ignition, and no live ejection during Layer 2 bridge development.
Only dummy loads, LEDs, PWM loggers, and servos may be connected after synthetic sensor input is verified.

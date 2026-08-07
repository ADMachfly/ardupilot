#include <AP_gtest.h>

/*
  HIL-F24-R3J: focused unit test for NavEKF3_core::average_accel_vector(),
  the pure helper backing InitialiseFilterBootstrap()'s bootstrap tilt-init
  accel averaging (AP_NavEKF3_core.cpp). It reduces sensitivity to a
  momentary startup-transient single accel sample by averaging over the
  existing ~1s bootstrap accumulation window instead.

  This is a static, state-free helper specifically so its sum/count
  averaging and zero-count guard can be verified in isolation, without
  needing a full NavEKF3_core/AP_DAL fixture.
 */

#include <AP_NavEKF3/AP_NavEKF3_core.h>

const AP_HAL::HAL& hal = AP_HAL::get_HAL();

TEST(NavEKF3BootstrapTiltInit, AveragesAccumulatedSamples)
{
    // three samples that average to a known vector
    Vector3F sum = Vector3F(3.0, -6.0, 9.0);
    Vector3F avg = NavEKF3_core::average_accel_vector(sum, 3);
    EXPECT_TRUE(is_equal(avg.x, ftype(1.0)));
    EXPECT_TRUE(is_equal(avg.y, ftype(-2.0)));
    EXPECT_TRUE(is_equal(avg.z, ftype(3.0)));
}

TEST(NavEKF3BootstrapTiltInit, SingleSampleIsUnchanged)
{
    // one sample -- the pre-fix behaviour -- must reproduce exactly
    Vector3F sum = Vector3F(131.34, -127.08, -983.16);
    Vector3F avg = NavEKF3_core::average_accel_vector(sum, 1);
    EXPECT_TRUE(is_equal(avg.x, ftype(131.34)));
    EXPECT_TRUE(is_equal(avg.y, ftype(-127.08)));
    EXPECT_TRUE(is_equal(avg.z, ftype(-983.16)));
}

TEST(NavEKF3BootstrapTiltInit, ZeroCountIsGuardedToZeroVector)
{
    // HIL-F24-R3J requirement 3: reject/guard a zero sample count safely --
    // must not divide by zero, must return a well-defined (zero) vector so
    // the caller's own fallback-to-instantaneous-sample path is reachable
    Vector3F sum = Vector3F(42.0, 42.0, 42.0);
    Vector3F avg = NavEKF3_core::average_accel_vector(sum, 0);
    EXPECT_TRUE(avg.is_zero());
}

TEST(NavEKF3BootstrapTiltInit, TransientSampleIsDilutedByLevelSamples)
{
    // Regression fixture from HIL_F24_R3G_ekf3_tilt_alignment_diagnosis.md:
    // a single transient off-level accel sample, [131.34, -127.08, -983.16]
    // mg, reproduces the real captured frozen AHRS2 attitude
    // (roll=7.365 deg, pitch=7.547 deg) exactly when used alone (the pre-fix
    // bug). Averaged alongside genuinely level samples across the
    // accumulation window (matching truth: [0, 0, -1000] mg), the resulting
    // vector must be much closer to level, proving the fix actually reduces
    // sensitivity to the transient rather than passing it straight through.
    const Vector3F transient(131.34, -127.08, -983.16);
    const Vector3F level(0.0, 0.0, -1000.0);

    // one transient sample diluted across a 20-sample window otherwise level
    // (representative of the ~1s window at a plausible EKF/IMU sample rate)
    Vector3F sum = transient;
    uint32_t count = 1;
    for (int i = 0; i < 19; i++) {
        sum += level;
        count++;
    }
    Vector3F avg = NavEKF3_core::average_accel_vector(sum, count);

    // pre-fix (single-sample) tilt from the transient alone:
    Vector3F single_normalized = transient;
    single_normalized.normalize();
    const ftype single_pitch_deg = degrees(asinF(single_normalized.x));

    // post-fix (averaged) tilt:
    Vector3F avg_normalized = avg;
    avg_normalized.normalize();
    const ftype avg_pitch_deg = degrees(asinF(avg_normalized.x));

    EXPECT_LT(fabsF(avg_pitch_deg), fabsF(single_pitch_deg));
    // diluted 20x -- expect well under 1 degree of residual pitch error
    EXPECT_LT(fabsF(avg_pitch_deg), ftype(1.0));
}

AP_GTEST_MAIN()

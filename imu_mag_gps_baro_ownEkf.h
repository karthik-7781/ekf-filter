#pragma once

#include <stdint.h>
#include <math.h>

#include <AP_Math/AP_Math.h>
#include <AP_Baro/AP_Baro.h>

/*
 * OurEKF - custom 22 state EKF for CubeOrange Plus / ArduPilot (ChibiOS)
 *
 * Conventions
 *   - Quaternion state is body -> NED.
 *   - quaternion_to_rotation_matrix() returns NED -> body.
 *   - x[2] is NED down position.
 *   - Barometer altitude is positive UP, so its EKF measurement is -height.
 */
class OurEKF
{
public:
    static constexpr int STATE_SIZE = 22;
    static constexpr int GPS_SIZE   = 3;
    static constexpr int MAG_SIZE   = 3;

    OurEKF();

    void init(const Vector3f &position,
              const Vector3f &velocity,
              const Quaternion &quaternion);

    void init_earth_field(const Vector3f &earth_field_ned,
                          float std_dev = 1.0f);

    static Quaternion attitude_from_accel_mag(const Vector3f &accel,
                                              const Vector3f &mag,
                                              float declination_rad);

    void predict(const Vector3f &accel,
                 const Vector3f &gyro,
                 float dt);

    bool update_gps(const Vector3f &gps_position);
    bool update_gps_velocity(const Vector3f &gps_velocity);
    bool update_mag(const Vector3f &mag);

    void update(const Vector3f &accel,
                const Vector3f &gyro,
                const Vector3f &mag,
                float dt,
                bool gps_available,
                const Vector3f &gps_position);

    void update(const Vector3f &accel,
                const Vector3f &gyro,
                const Vector3f &mag,
                float dt,
                bool gps_available,
                const Vector3f &gps_position,
                bool gps_vel_available,
                const Vector3f &gps_velocity);

    // ---------------------------------------------------------------------
    // Configuration
    // ---------------------------------------------------------------------

    void set_body_mag_learning(bool enable);
    void set_tilt_fusion(bool enable) { tilt_fusion_enabled = enable; }
    void set_mag_noise(float std_dev) { mag_noise_std = std_dev; }

    // Barometer measurement noise, metres 1-sigma.
    void set_baro_noise(float std_dev) { baro_noise_std = std_dev; }

    // ---------------------------------------------------------------------
    // State getters
    // ---------------------------------------------------------------------

    Vector3f get_position() const;
    Vector3f get_velocity() const;
    Quaternion get_quaternion() const;

    Vector3f get_accel_bias() const;
    Vector3f get_gyro_bias() const;

    Vector3f get_accel_body() const;
    Vector3f get_accel_world() const;

    float get_position_x() const;
    float get_position_y() const;
    float get_position_z() const;

    float get_velocity_x() const;
    float get_velocity_y() const;
    float get_velocity_z() const;

    float get_accel_bias_x() const;
    float get_accel_bias_y() const;
    float get_accel_bias_z() const;

    float get_gyro_bias_x() const;
    float get_gyro_bias_y() const;
    float get_gyro_bias_z() const;

    void Orientation();

    float get_roll() const;
    float get_pitch() const;
    float get_yaw() const;

    void get_state(float state[STATE_SIZE]) const;
    void get_covariance(float covariance[STATE_SIZE][STATE_SIZE]) const;

    float get_position_variance_z() const;

    float get_innovation_x() const;
    float get_innovation_y() const;
    float get_innovation_z() const;

    Vector3f get_velocity_innovation() const;
    Vector3f get_mag_innovation() const;

    bool is_initialized() const;
    bool is_healthy() const { return healthy; }
    uint16_t get_reset_count() const { return reset_count; }

    Vector3f last_accel_body;
    Vector3f last_accel_world;

    // Raw barometric altitude, positive UP, in metres.
    // The first 100 valid samples after init() are averaged as the bench zero.
    bool update_baro(float baro_alt_raw);

    // Returns the most recently established bench-zero offset.
    float get_baro_offset() const { return baro_offset; }

    // Returns true after the bench-zero calibration has completed.
    bool is_baro_calibrated() const { return have_baro_offset; }

    float get_baro_altitude() const;

    uint32_t last_baro_sample_ms;


    float get_baro_raw() const;
    float get_baro_innovation() const;
    uint8_t get_baro_fusion_result() const;
private:
    enum FuseResult : uint8_t {
        FUSE_OK = 0,
        FUSE_REJECTED,
        FUSE_INVALID
    };

    // ---------------------------------------------------------------------
    // State
    // ---------------------------------------------------------------------

    float x[STATE_SIZE];

    float roll;
    float pitch;
    float yaw;

    // ---------------------------------------------------------------------
    // Covariance and prediction matrices
    // ---------------------------------------------------------------------

    float P[STATE_SIZE][STATE_SIZE];
    float F[STATE_SIZE][STATE_SIZE];
    float Q[STATE_SIZE][STATE_SIZE];

    // Scalar fusion work. Column 0 is used by fuse_scalar().
    float K[STATE_SIZE][GPS_SIZE];
    float PHt[STATE_SIZE][GPS_SIZE];

    float innovation[GPS_SIZE];
    float innovation_vel[GPS_SIZE];
    float innovation_mag[MAG_SIZE];

    // Persistent work buffers. These are class members so they do not use
    // the ChibiOS task stack.
    float cov_temp1[STATE_SIZE][STATE_SIZE];
    float cov_temp2[STATE_SIZE][STATE_SIZE];

    // ---------------------------------------------------------------------
    // Status / configuration
    // ---------------------------------------------------------------------

    bool initialized;
    bool healthy;
    uint16_t reset_count;

    Vector3f last_gps_position;
    bool have_last_gps;

    bool mag_learning;
    bool tilt_fusion_enabled;
    float mag_noise_std;

    bool have_earth_seed;
    Vector3f earth_seed;
    float earth_seed_std;

    // ---------------------------------------------------------------------
    // Prediction / covariance
    // ---------------------------------------------------------------------

    void build_F(const Vector3f &accel_body,
                 const Vector3f &gyro_body,
                 float dt);

    void build_Q(float dt);
    void predict_covariance();

    // ---------------------------------------------------------------------
    // Generic scalar fusion
    // ---------------------------------------------------------------------

    FuseResult fuse_scalar(const float *Hrow,
                           float innov,
                           float r_var,
                           float gate_sigma);

    void fuse_accel_tilt(const Vector3f &accel_raw);

    // ---------------------------------------------------------------------
    // Magnetometer model
    // ---------------------------------------------------------------------

    Vector3f predict_magnetometer(const Quaternion &q) const;

    // ---------------------------------------------------------------------
    // Quaternion
    // ---------------------------------------------------------------------

    Quaternion get_state_quaternion() const;
    void set_state_quaternion(const Quaternion &q);

    Quaternion quaternion_from_gyro(const Vector3f &gyro,
                                    float dt) const;

    void propagate_quaternion(const Vector3f &gyro,
                              float dt);

    Matrix3f quaternion_to_rotation_matrix(const Quaternion &q) const;

    void normalize_quaternion();

    // ---------------------------------------------------------------------
    // Housekeeping
    // ---------------------------------------------------------------------

    void force_symmetry_and_constrain();
    void constrain_states();
    bool state_is_finite() const;
    void reset_after_divergence();
    void apply_earth_seed();

    // ---------------------------------------------------------------------
    // Matrix helpers
    // ---------------------------------------------------------------------

    void zero_matrix(float A[STATE_SIZE][STATE_SIZE]);

    // ---------------------------------------------------------------------
    // Barometer
    // ---------------------------------------------------------------------

    // baro_noise_std is 1-sigma measurement noise in metres.
    float baro_noise_std;

    // Bench-zero reference.
    bool have_baro_offset;
    float baro_offset;

    // First 100 valid samples are averaged while the board is stationary.
    bool baro_calibrating;
    uint16_t baro_calib_count;
    float baro_calib_sum;


    float baro_raw;
    float baro_innovation;
    float baro_fusion_result;
};

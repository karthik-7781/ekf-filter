#pragma once

#include <stdint.h>
#include <math.h>

#include <AP_Math/AP_Math.h>
#include <AP_Compass/AP_Compass.h>

class OurEKF
{
public:

    static constexpr int STATE_SIZE = 16;
    static constexpr int GPS_SIZE   = 3;
    static constexpr int MAG_SIZE = 1;

    OurEKF();

    float cov_temp1[16][16];
    float cov_temp2[16][16];

    /*
     * Initialize EKF.
     *
     * position:
     *     Initial position [m]
     *
     * velocity:
     *     Initial velocity [m/s]
     *
     * quaternion:
     *     Initial attitude
     */
    void init(const Vector3f &position,
              const Vector3f &velocity,
              const Quaternion &quaternion);

    /*
     * Prediction step.
     *
     * accel:
     *     Raw accelerometer measurement
     *
     * gyro:
     *     Raw gyroscope measurement
     *
     * dt:
     *     Time step in seconds
     */
    void predict(const Vector3f &accel,
                 const Vector3f &gyro,
                 float dt);

    /*
     * GPS position measurement update.
     *
     * gps_position:
     *     GPS position in the SAME local coordinate frame
     *     and units as the EKF state.
     */
    bool update_gps(const Vector3f &gps_position);
    bool update_mag(const Vector3f &mag);

    /*
     * Complete EKF update.
     *
     * Prediction is performed every time.
     * GPS update is performed only when gps_available is true.
     */
    void update(const Vector3f &accel,
                const Vector3f &gyro,
                const Vector3f &mag,
                float dt,
                bool gps_available,
                const Vector3f &gps_position);

    // ---------------------------------------------------------------------
    // State getters
    // ---------------------------------------------------------------------

    Vector3f last_accel_body ;
    Vector3f last_accel_world;
    Vector3f get_accel_body() const;
    Vector3f get_accel_world() const;
    Vector3f get_position() const;
    Vector3f get_velocity() const;
    Quaternion get_quaternion() const;

    Vector3f get_accel_bias() const;
    Vector3f get_gyro_bias() const;


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
    

    /*
     * Copy complete 16-state vector.
     */
    void get_state(float state[STATE_SIZE]) const;

    /*
     * Copy covariance matrix.
     */
    void get_covariance(float covariance[STATE_SIZE][STATE_SIZE]) const;

    /*
     * Useful for debugging.
     */
    float get_innovation_x() const;
    float get_innovation_y() const;
    float get_innovation_z() const;

    bool is_initialized() const;



private:

    // ---------------------------------------------------------------------
    // State vector
    //
    // x[0]  = px
    // x[1]  = py
    // x[2]  = pz
    //
    // x[3]  = vx
    // x[4]  = vy
    // x[5]  = vz
    //
    // x[6]  = qw
    // x[7]  = qx
    // x[8]  = qy
    // x[9]  = qz
    //
    // x[10] = bax
    // x[11] = bay
    // x[12] = baz
    //
    // x[13] = bgx
    // x[14] = bgy
    // x[15] = bgz
    // ---------------------------------------------------------------------

    float roll;
    float pitch;
    float yaw;

    float x[STATE_SIZE];

    // Covariance
    float P[STATE_SIZE][STATE_SIZE];

    // State transition Jacobian
    float F[STATE_SIZE][STATE_SIZE];

    // Process noise covariance
    float Q[STATE_SIZE][STATE_SIZE];

    // Measurement Jacobian
    float H[GPS_SIZE][STATE_SIZE];

    // float H[MAG_SIZE][STATE_SIZE];

    // GPS measurement covariance
    float R[GPS_SIZE][GPS_SIZE];

    // float R[MAG_SIZE][STATE_SIZE];

    // Innovation covariance
    float S[GPS_SIZE][GPS_SIZE];

    // float S[MAG_SIZE][STATE_SIZE];

    // Kalman gain
    float K[STATE_SIZE][GPS_SIZE];
    // float K[STATE_SIZE][MAG_SIZE];

    // Innovation
    float innovation[GPS_SIZE];


    // Magnetometer yaw update
    float H_mag[MAG_SIZE][STATE_SIZE];
    float R_mag[MAG_SIZE][MAG_SIZE];
    float S_mag[MAG_SIZE][MAG_SIZE];
    float K_mag[STATE_SIZE][MAG_SIZE];
    float innovation_mag[MAG_SIZE];


    bool initialized;

    // Calibration accumulators (stationary accel calibration)
    bool calib_active;
    int  calib_target;
    int  calib_count;
    Vector3f calib_sum_measured;
    Vector3f calib_sum_expected;

    // Internal functions
 

    void reset_matrices();

    void build_F(const Vector3f &accel_body,
                 float dt);

    void build_Q(float dt);

    void predict_covariance();

    void build_H();

    void build_R();

    bool calculate_innovation_covariance();

    bool calculate_kalman_gain();

    void correct_state();

    void correct_covariance();

    void normalize_quaternion();

   

   
    // Quaternion functions
   

    Quaternion get_state_quaternion() const;

    void set_state_quaternion(const Quaternion &q);

    Quaternion quaternion_from_gyro(const Vector3f &gyro,
                                    float dt) const;

    void propagate_quaternion(const Vector3f &gyro,
                              float dt);

    Matrix3f quaternion_to_rotation_matrix(const Quaternion &q) const;

    // Small matrix helpers


    bool inverse_3x3(const float A[3][3],
                     float A_inv[3][3]) const;

    void zero_matrix_16(float A[16][16]);

    void zero_matrix_16x3(float A[16][3]);

    void zero_matrix_3x16(float A[3][16]);

    void zero_matrix_3(float A[3][3]);

    void identity_matrix_16(float A[16][16]);
};

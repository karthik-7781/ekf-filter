// #pragma once

// #include <stdint.h>
// #include <math.h>

// #include <AP_Math/AP_Math.h>
// #include <AP_Compass/AP_Compass.h>

// class OurEKF
// {
// public:

//     static constexpr int STATE_SIZE = 22;
//     static constexpr int GPS_SIZE   = 3;
//     static constexpr int MAG_SIZE = 1;

//     OurEKF();

//     float cov_temp1[STATE_SIZE][STATE_SIZE];
//     float cov_temp2[STATE_SIZE][STATE_SIZE];

//     /*
//      * Initialize EKF.
//      *
//      * position:
//      *     Initial position [m]
//      *
//      * velocity:
//      *     Initial velocity [m/s]
//      *
//      * quaternion:
//      *     Initial attitude
//      */
//     void init(const Vector3f &position,
//               const Vector3f &velocity,
//               const Quaternion &quaternion);

//     /*
//      * Prediction step.
//      *
//      * accel:
//      *     Raw accelerometer measurement
//      *
//      * gyro:
//      *     Raw gyroscope measurement
//      *
//      * dt:
//      *     Time step in seconds
//      */
//     void predict(const Vector3f &accel,
//                  const Vector3f &gyro,
//                  float dt);

//     /*
//      * GPS position measurement update.
//      *
//      * gps_position:
//      *     GPS position in the SAME local coordinate frame
//      *     and units as the EKF state.
//      */
//     bool update_gps(const Vector3f &gps_position);
//     bool update_mag(const Vector3f &mag);

//     /*
//      * Complete EKF update.
//      *
//      * Prediction is performed every time.
//      * GPS update is performed only when gps_available is true.
//      */
//     void update(const Vector3f &accel,
//                 const Vector3f &gyro,
//                 const Vector3f &mag,
//                 float dt,
//                 bool gps_available,
//                 const Vector3f &gps_position);

//     // ---------------------------------------------------------------------
//     // State getters
//     // ---------------------------------------------------------------------

//     Vector3f last_accel_body ;
//     Vector3f last_accel_world;
//     Vector3f get_accel_body() const;
//     Vector3f get_accel_world() const;
//     Vector3f get_position() const;
//     Vector3f get_velocity() const;
//     Quaternion get_quaternion() const;

//     Vector3f get_accel_bias() const;
//     Vector3f get_gyro_bias() const;


//     float get_position_x() const;
//     float get_position_y() const;
//     float get_position_z() const;

//     float get_velocity_x() const;
//     float get_velocity_y() const;
//     float get_velocity_z() const;

//     float get_accel_bias_x() const;
//     float get_accel_bias_y() const;
//     float get_accel_bias_z() const;

//     float get_gyro_bias_x() const;
//     float get_gyro_bias_y() const;
//     float get_gyro_bias_z() const;


//     void Orientation();
    

//     float get_roll() const;
//     float get_pitch() const;    
//     float get_yaw() const;
    

//     /*
//      * Copy complete 16-state vector.
//      */
//     void get_state(float state[STATE_SIZE]) const;

//     /*
//      * Copy covariance matrix.
//      */
//     void get_covariance(float covariance[STATE_SIZE][STATE_SIZE]) const;

//     /*
//      * Useful for debugging.
//      */
//     float get_innovation_x() const;
//     float get_innovation_y() const;
//     float get_innovation_z() const;

//     bool is_initialized() const;



// private:

//     // ---------------------------------------------------------------------
//     // State vector
//     //
//     // x[0]  = px
//     // x[1]  = py
//     // x[2]  = pz
//     //
//     // x[3]  = vx
//     // x[4]  = vy
//     // x[5]  = vz
//     //
//     // x[6]  = qw
//     // x[7]  = qx
//     // x[8]  = qy
//     // x[9]  = qz
//     //
//     // x[10] = bax
//     // x[11] = bay
//     // x[12] = baz
//     //
//     // x[13] = bgx
//     // x[14] = bgy
//     // x[15] = bgz

//     //x[16]= Earths magnetic field x
//     //x[17]= Earths magnetic field y
//     //x[18]= Earths magnetic field z

//     //x[19]= Body magnetic field x
//     //x[20]= Body magnetic field y
//     //x[21]= Body magnetic field z
//     // ---------------------------------------------------------------------

//     float roll;
//     float pitch;
//     float yaw;

//     float x[STATE_SIZE];

//     // Covariance
//     float P[STATE_SIZE][STATE_SIZE];

//     // State transition Jacobian
//     float F[STATE_SIZE][STATE_SIZE];

//     // Process noise covariance
//     float Q[STATE_SIZE][STATE_SIZE];

//     // Measurement Jacobian
//     float H[GPS_SIZE][STATE_SIZE];

//     // float H[MAG_SIZE][STATE_SIZE];

//     // GPS measurement covariance
//     float R[GPS_SIZE][GPS_SIZE];

//     // float R[MAG_SIZE][STATE_SIZE];

//     // Innovation covariance
//     float S[GPS_SIZE][GPS_SIZE];

//     // float S[MAG_SIZE][STATE_SIZE];

//     // Kalman gain
//     float K[STATE_SIZE][GPS_SIZE];
//     // float K[STATE_SIZE][MAG_SIZE];

//     // Innovation
//     float innovation[GPS_SIZE];


//     // Magnetometer yaw update
//     float H_mag[MAG_SIZE][STATE_SIZE];
//     float R_mag[MAG_SIZE][MAG_SIZE];
//     float S_mag[MAG_SIZE][MAG_SIZE];
//     float K_mag[STATE_SIZE][MAG_SIZE];
//     float innovation_mag[MAG_SIZE];


//     bool initialized;

//     // Calibration accumulators (stationary accel calibration)
//     bool calib_active;
//     int  calib_target;
//     int  calib_count;
//     Vector3f calib_sum_measured;
//     Vector3f calib_sum_expected;

//     // Internal functions
 

//     void reset_matrices();

//     void build_F(const Vector3f &accel_body,
//                  float dt);

//     void build_Q(float dt);

//     void predict_covariance();

//     void build_H();

//     void build_R();

//     bool calculate_innovation_covariance();

//     bool calculate_kalman_gain();

//     void correct_state();

//     void correct_covariance();

//     void normalize_quaternion();

   

   
//     // Quaternion functions
   

//     Quaternion get_state_quaternion() const;

//     void set_state_quaternion(const Quaternion &q);

//     Quaternion quaternion_from_gyro(const Vector3f &gyro,
//                                     float dt) const;

//     void propagate_quaternion(const Vector3f &gyro,
//                               float dt);

//     Matrix3f quaternion_to_rotation_matrix(const Quaternion &q) const;

//     // Small matrix helpers


//     bool inverse_3x3(const float A[3][3],
//                      float A_inv[3][3]) const;

//     void zero_matrix_16(float A[STATE_SIZE][STATE_SIZE]);

//     void zero_matrix_16x3(float A[STATE_SIZE][3]);

//     void zero_matrix_3x16(float A[3][STATE_SIZE]);

//     void zero_matrix_3(float A[3][3]);

//     void identity_matrix_16(float A[STATE_SIZE][STATE_SIZE]);
// };

// #pragma once

// #include <stdint.h>
// #include <math.h>

// #include <AP_Math/AP_Math.h>

// class OurEKF
// {
// public:

//     /*
//      * State:
//      *
//      *  0  px
//      *  1  py
//      *  2  pz
//      *
//      *  3  vx
//      *  4  vy
//      *  5  vz
//      *
//      *  6  qw
//      *  7  qx
//      *  8  qy
//      *  9  qz
//      *
//      * 10  accel bias x
//      * 11  accel bias y
//      * 12  accel bias z
//      *
//      * 13  gyro bias x
//      * 14  gyro bias y
//      * 15  gyro bias z
//      *
//      * 16  earth magnetic field x
//      * 17  earth magnetic field y
//      * 18  earth magnetic field z
//      *
//      * 19  body magnetic field x
//      * 20  body magnetic field y
//      * 21  body magnetic field z
//      */

//     static constexpr int STATE_SIZE = 22;
//     static constexpr int GPS_SIZE   = 3;
//     static constexpr int MAG_SIZE   = 3;

//     Vector3f last_gps_position;
//     bool have_last_gps;

//     OurEKF();

//     // ---------------------------------------------------------------------
//     // Main interface
//     // ---------------------------------------------------------------------

//     void init(const Vector3f &position,
//               const Vector3f &velocity,
//               const Quaternion &quaternion);

//     void predict(const Vector3f &accel,
//                  const Vector3f &gyro,
//                  float dt);

//     bool update_gps(const Vector3f &gps_position);

//     bool update_mag(const Vector3f &mag);

//     void update(const Vector3f &accel,
//                 const Vector3f &gyro,
//                 const Vector3f &mag,
//                 float dt,
//                 bool gps_available,
//                 const Vector3f &gps_position);

//     // ---------------------------------------------------------------------
//     // State getters
//     // ---------------------------------------------------------------------

//     Vector3f get_accel_body() const;
//     Vector3f get_accel_world() const;

//     Vector3f get_position() const;
//     Vector3f get_velocity() const;
//     Quaternion get_quaternion() const;

//     Vector3f get_accel_bias() const;
//     Vector3f get_gyro_bias() const;

//     float get_position_x() const;
//     float get_position_y() const;
//     float get_position_z() const;

//     float get_velocity_x() const;
//     float get_velocity_y() const;
//     float get_velocity_z() const;

//     float get_accel_bias_x() const;
//     float get_accel_bias_y() const;
//     float get_accel_bias_z() const;

//     float get_gyro_bias_x() const;
//     float get_gyro_bias_y() const;
//     float get_gyro_bias_z() const;

//     void Orientation();

//     float get_roll() const;
//     float get_pitch() const;
//     float get_yaw() const;

//     void get_state(float state[STATE_SIZE]) const;

//     void get_covariance(float covariance[STATE_SIZE][STATE_SIZE]) const;

//     float get_innovation_x() const;
//     float get_innovation_y() const;
//     float get_innovation_z() const;

//     bool is_initialized() const;

//     Vector3f last_accel_body;
//     Vector3f last_accel_world;

// private:

//     // ---------------------------------------------------------------------
//     // State
//     // ---------------------------------------------------------------------

//     float x[STATE_SIZE];

//     float roll;
//     float pitch;
//     float yaw;

//     // ---------------------------------------------------------------------
//     // EKF covariance / matrices
//     // ---------------------------------------------------------------------

//     float P[STATE_SIZE][STATE_SIZE];
//     float F[STATE_SIZE][STATE_SIZE];
//     float Q[STATE_SIZE][STATE_SIZE];

//     // GPS
//     float H[GPS_SIZE][STATE_SIZE];
//     float R[GPS_SIZE][GPS_SIZE];
//     float S[GPS_SIZE][GPS_SIZE];
//     float K[STATE_SIZE][GPS_SIZE];
//     float innovation[GPS_SIZE];

//     // Magnetometer
//     float H_mag[MAG_SIZE][STATE_SIZE];
//     float R_mag[MAG_SIZE][MAG_SIZE];
//     float S_mag[MAG_SIZE][MAG_SIZE];
//     float K_mag[STATE_SIZE][MAG_SIZE];
//     float innovation_mag[MAG_SIZE];

//     // ---------------------------------------------------------------------
//     // Persistent work buffers.
//     //
//     // IMPORTANT:
//     // These are class members rather than local arrays.
//     // This prevents 22x22 matrices from being placed on the task stack.
//     // ---------------------------------------------------------------------

//     float cov_temp1[STATE_SIZE][STATE_SIZE];
//     float cov_temp2[STATE_SIZE][STATE_SIZE];

//     float PHt[STATE_SIZE][GPS_SIZE];

//     // ---------------------------------------------------------------------
//     // Status
//     // ---------------------------------------------------------------------

//     bool initialized;

//     // ---------------------------------------------------------------------
//     // Calibration
//     // ---------------------------------------------------------------------

//     bool calib_active;
//     int  calib_target;
//     int  calib_count;

//     Vector3f calib_sum_measured;
//     Vector3f calib_sum_expected;

//     // ---------------------------------------------------------------------
//     // Matrix helpers
//     // ---------------------------------------------------------------------

//     void zero_matrix(float A[STATE_SIZE][STATE_SIZE]);

//     void zero_matrix_x3(float A[STATE_SIZE][3]);

//     void zero_matrix_3x(float A[3][STATE_SIZE]);

//     void zero_matrix_3(float A[3][3]);

//     void identity_matrix(float A[STATE_SIZE][STATE_SIZE]);

//     // ---------------------------------------------------------------------
//     // Prediction
//     // ---------------------------------------------------------------------

//     void build_F(const Vector3f &accel_body,
//                  float dt);

//     void build_Q(float dt);

//     void predict_covariance();

//     // ---------------------------------------------------------------------
//     // GPS
//     // ---------------------------------------------------------------------

//     void build_H();

//     void build_R();

//     bool calculate_innovation_covariance();

//     bool calculate_kalman_gain();

//     void correct_state();

//     void correct_covariance();

//     // ---------------------------------------------------------------------
//     // Magnetometer
//     // ---------------------------------------------------------------------

//     Vector3f predict_magnetometer(const Quaternion &q) const;

//     void build_mag_measurement_jacobian(
//         const Vector3f &mag_prediction);

//     bool fuse_mag_component(
//         const Vector3f &mag,
//         uint8_t component);

//     // ---------------------------------------------------------------------
//     // Quaternion
//     // ---------------------------------------------------------------------

//     Quaternion get_state_quaternion() const;

//     void set_state_quaternion(const Quaternion &q);

//     Quaternion quaternion_from_gyro(const Vector3f &gyro,
//                                     float dt) const;

//     void propagate_quaternion(const Vector3f &gyro,
//                               float dt);

//     Matrix3f quaternion_to_rotation_matrix(
//         const Quaternion &q) const;

//     void normalize_quaternion();

//     // ---------------------------------------------------------------------
//     // Small matrix
//     // ---------------------------------------------------------------------

//     bool inverse_3x3(const float A[3][3],
//                      float A_inv[3][3]) const;
// };












// // #pragma once

// // #include <stdint.h>
// // #include <math.h>

// // #include <AP_Math/AP_Math.h>
// // #include <AP_Compass/AP_Compass.h>

// // class OurEKF
// // {
// // public:

// //     static constexpr int STATE_SIZE = 16;
// //     static constexpr int GPS_SIZE   = 3;
// //     static constexpr int MAG_SIZE = 1;

// //     OurEKF();

// //     float cov_temp1[16][16];
// //     float cov_temp2[16][16];

// //     /*
// //      * Initialize EKF.
// //      *
// //      * position:
// //      *     Initial position [m]
// //      *
// //      * velocity:
// //      *     Initial velocity [m/s]
// //      *
// //      * quaternion:
// //      *     Initial attitude
// //      */
// //     void init(const Vector3f &position,
// //               const Vector3f &velocity,
// //               const Quaternion &quaternion);

// //     /*
// //      * Prediction step.
// //      *
// //      * accel:
// //      *     Raw accelerometer measurement
// //      *
// //      * gyro:
// //      *     Raw gyroscope measurement
// //      *
// //      * dt:
// //      *     Time step in seconds
// //      */
// //     void predict(const Vector3f &accel,
// //                  const Vector3f &gyro,
// //                  float dt);

// //     /*
// //      * GPS position measurement update.
// //      *
// //      * gps_position:
// //      *     GPS position in the SAME local coordinate frame
// //      *     and units as the EKF state.
// //      */
// //     bool update_gps(const Vector3f &gps_position);
// //     bool update_mag(const Vector3f &mag);

// //     /*
// //      * Complete EKF update.
// //      *
// //      * Prediction is performed every time.
// //      * GPS update is performed only when gps_available is true.
// //      */
// //     void update(const Vector3f &accel,
// //                 const Vector3f &gyro,
// //                 const Vector3f &mag,
// //                 float dt,
// //                 bool gps_available,
// //                 const Vector3f &gps_position);

// //     // ---------------------------------------------------------------------
// //     // State getters
// //     // ---------------------------------------------------------------------

// //     Vector3f last_accel_body ;
// //     Vector3f last_accel_world;
// //     Vector3f get_accel_body() const;
// //     Vector3f get_accel_world() const;
// //     Vector3f get_position() const;
// //     Vector3f get_velocity() const;
// //     Quaternion get_quaternion() const;

// //     Vector3f get_accel_bias() const;
// //     Vector3f get_gyro_bias() const;


// //     float get_position_x() const;
// //     float get_position_y() const;
// //     float get_position_z() const;

// //     float get_velocity_x() const;
// //     float get_velocity_y() const;
// //     float get_velocity_z() const;

// //     float get_accel_bias_x() const;
// //     float get_accel_bias_y() const;
// //     float get_accel_bias_z() const;

// //     float get_gyro_bias_x() const;
// //     float get_gyro_bias_y() const;
// //     float get_gyro_bias_z() const;


// //     void Orientation();
    

// //     float get_roll() const;
// //     float get_pitch() const;    
// //     float get_yaw() const;
    

// //     /*
// //      * Copy complete 16-state vector.
// //      */
// //     void get_state(float state[STATE_SIZE]) const;

// //     /*
// //      * Copy covariance matrix.
// //      */
// //     void get_covariance(float covariance[STATE_SIZE][STATE_SIZE]) const;

// //     /*
// //      * Useful for debugging.
// //      */
// //     float get_innovation_x() const;
// //     float get_innovation_y() const;
// //     float get_innovation_z() const;

// //     bool is_initialized() const;



// // private:

// //     // ---------------------------------------------------------------------
// //     // State vector
// //     //
// //     // x[0]  = px
// //     // x[1]  = py
// //     // x[2]  = pz
// //     //
// //     // x[3]  = vx
// //     // x[4]  = vy
// //     // x[5]  = vz
// //     //
// //     // x[6]  = qw
// //     // x[7]  = qx
// //     // x[8]  = qy
// //     // x[9]  = qz
// //     //
// //     // x[10] = bax
// //     // x[11] = bay
// //     // x[12] = baz
// //     //
// //     // x[13] = bgx
// //     // x[14] = bgy
// //     // x[15] = bgz
// //     // ---------------------------------------------------------------------

// //     float roll;
// //     float pitch;
// //     float yaw;

// //     float x[STATE_SIZE];

// //     // Covariance
// //     float P[STATE_SIZE][STATE_SIZE];

// //     // State transition Jacobian
// //     float F[STATE_SIZE][STATE_SIZE];

// //     // Process noise covariance
// //     float Q[STATE_SIZE][STATE_SIZE];

// //     // Measurement Jacobian
// //     float H[GPS_SIZE][STATE_SIZE];

// //     // float H[MAG_SIZE][STATE_SIZE];

// //     // GPS measurement covariance
// //     float R[GPS_SIZE][GPS_SIZE];

// //     // float R[MAG_SIZE][STATE_SIZE];

// //     // Innovation covariance
// //     float S[GPS_SIZE][GPS_SIZE];

// //     // float S[MAG_SIZE][STATE_SIZE];

// //     // Kalman gain
// //     float K[STATE_SIZE][GPS_SIZE];
// //     // float K[STATE_SIZE][MAG_SIZE];

// //     // Innovation
// //     float innovation[GPS_SIZE];


// //     // Magnetometer yaw update
// //     float H_mag[MAG_SIZE][STATE_SIZE];
// //     float R_mag[MAG_SIZE][MAG_SIZE];
// //     float S_mag[MAG_SIZE][MAG_SIZE];
// //     float K_mag[STATE_SIZE][MAG_SIZE];
// //     float innovation_mag[MAG_SIZE];


// //     bool initialized;

// //     // Calibration accumulators (stationary accel calibration)
// //     bool calib_active;
// //     int  calib_target;
// //     int  calib_count;
// //     Vector3f calib_sum_measured;
// //     Vector3f calib_sum_expected;

// //     // Internal functions
 

// //     void reset_matrices();

// //     void build_F(const Vector3f &accel_body,
// //                  float dt);

// //     void build_Q(float dt);

// //     void predict_covariance();

// //     void build_H();

// //     void build_R();

// //     bool calculate_innovation_covariance();

// //     bool calculate_kalman_gain();

// //     void correct_state();

// //     void correct_covariance();

// //     void normalize_quaternion();

   

   
// //     // Quaternion functions
   

// //     Quaternion get_state_quaternion() const;

// //     void set_state_quaternion(const Quaternion &q);

// //     Quaternion quaternion_from_gyro(const Vector3f &gyro,
// //                                     float dt) const;

// //     void propagate_quaternion(const Vector3f &gyro,
// //                               float dt);

// //     Matrix3f quaternion_to_rotation_matrix(const Quaternion &q) const;

// //     // Small matrix helpers


// //     bool inverse_3x3(const float A[3][3],
// //                      float A_inv[3][3]) const;

// //     void zero_matrix_16(float A[16][16]);

// //     void zero_matrix_16x3(float A[16][3]);

// //     void zero_matrix_3x16(float A[3][16]);

// //     void zero_matrix_3(float A[3][3]);

// //     void identity_matrix_16(float A[16][16]);
// // };


// #pragma once

// #include <stdint.h>
// #include <math.h>

// #include <AP_Math/AP_Math.h>

// class OurEKF
// {
// public:
//     // ---------------------------------------------------------------------
//     // State layout
//     // ---------------------------------------------------------------------
//     // 0..2   position NED [m]
//     // 3..5   velocity NED [m/s]
//     // 6..9   quaternion [qw qx qy qz]
//     // 10..12 accelerometer bias [m/s^2]
//     // 13..15 gyro bias [rad/s]
//     // 16..18 earth magnetic field [sensor units]
//     // 19..21 body magnetic bias/field [sensor units]
//     // ---------------------------------------------------------------------
//     static constexpr int STATE_SIZE = 22;
//     static constexpr int GPS_SIZE   = 3;
//     static constexpr int MAG_SIZE   = 3;

//     OurEKF();

//     void init(const Vector3f &position,
//               const Vector3f &velocity,
//               const Quaternion &quaternion);

//     void predict(const Vector3f &accel,
//                  const Vector3f &gyro,
//                  float dt);

//     bool update_gps(const Vector3f &gps_position);
//     bool update_mag(const Vector3f &mag);

//     void update(const Vector3f &accel,
//                 const Vector3f &gyro,
//                 const Vector3f &mag,
//                 float dt,
//                 bool gps_available,
//                 const Vector3f &gps_position);

//     // ---------------------------------------------------------------------
//     // State getters
//     // ---------------------------------------------------------------------
//     Vector3f get_position() const;
//     Vector3f get_velocity() const;
//     Quaternion get_quaternion() const;

//     Vector3f get_accel_bias() const;
//     Vector3f get_gyro_bias() const;

//     Vector3f get_accel_body() const;
//     Vector3f get_accel_world() const;

//     float get_position_x() const;
//     float get_position_y() const;
//     float get_position_z() const;

//     float get_velocity_x() const;
//     float get_velocity_y() const;
//     float get_velocity_z() const;

//     float get_accel_bias_x() const;
//     float get_accel_bias_y() const;
//     float get_accel_bias_z() const;

//     float get_gyro_bias_x() const;
//     float get_gyro_bias_y() const;
//     float get_gyro_bias_z() const;

//     void Orientation();

//     float get_roll() const;
//     float get_pitch() const;
//     float get_yaw() const;

//     void get_state(float state[STATE_SIZE]) const;
//     void get_covariance(float covariance[STATE_SIZE][STATE_SIZE]) const;

//     float get_innovation_x() const;
//     float get_innovation_y() const;
//     float get_innovation_z() const;

//     bool is_initialized() const;

//     // Last corrected accelerometer values, useful for debugging/logging.
//     Vector3f last_accel_body;
//     Vector3f last_accel_world;

// private:
//     // ---------------------------------------------------------------------
//     // State
//     // ---------------------------------------------------------------------
//     float x[STATE_SIZE];

//     float roll;
//     float pitch;
//     float yaw;

//     // ---------------------------------------------------------------------
//     // Covariance and prediction matrices
//     // ---------------------------------------------------------------------
//     float P[STATE_SIZE][STATE_SIZE];
//     float F[STATE_SIZE][STATE_SIZE];
//     float Q[STATE_SIZE][STATE_SIZE];

//     // ---------------------------------------------------------------------
//     // GPS measurement matrices
//     // ---------------------------------------------------------------------
//     float H[GPS_SIZE][STATE_SIZE];
//     float R[GPS_SIZE][GPS_SIZE];
//     float S[GPS_SIZE][GPS_SIZE];
//     float K[STATE_SIZE][GPS_SIZE];
//     float innovation[GPS_SIZE];

//     // ---------------------------------------------------------------------
//     // Magnetometer measurement matrices
//     // ---------------------------------------------------------------------
//     float H_mag[MAG_SIZE][STATE_SIZE];
//     float R_mag[MAG_SIZE][MAG_SIZE];
//     float S_mag[MAG_SIZE][MAG_SIZE];
//     float K_mag[STATE_SIZE][MAG_SIZE];
//     float innovation_mag[MAG_SIZE];

//     // ---------------------------------------------------------------------
//     // Persistent work buffers.
//     // Keep these as members, not local arrays, because this is running on
//     // the CubeOrangePlus ChibiOS task stack.
//     // ---------------------------------------------------------------------
//     float cov_temp1[STATE_SIZE][STATE_SIZE];
//     float cov_temp2[STATE_SIZE][STATE_SIZE];
//     float PHt[STATE_SIZE][GPS_SIZE];

//     // ---------------------------------------------------------------------
//     // Status
//     // ---------------------------------------------------------------------
//     bool initialized;

//     Vector3f last_gps_position;
//     bool have_last_gps;

//     // ---------------------------------------------------------------------
//     // Calibration state retained for compatibility with the existing .cpp.
//     // ---------------------------------------------------------------------
//     bool calib_active;
//     int calib_target;
//     int calib_count;
//     Vector3f calib_sum_measured;
//     Vector3f calib_sum_expected;

//     // ---------------------------------------------------------------------
//     // Prediction / covariance
//     // ---------------------------------------------------------------------
//     void build_F(const Vector3f &accel_body, float dt);
//     void build_Q(float dt);
//     void predict_covariance();

//     // ---------------------------------------------------------------------
//     // GPS fusion
//     // ---------------------------------------------------------------------
//     void build_H();
//     void build_R();
//     bool calculate_innovation_covariance();
//     bool calculate_kalman_gain();
//     void correct_state();
//     void correct_covariance();

//     // ---------------------------------------------------------------------
//     // Magnetometer fusion
//     // ---------------------------------------------------------------------
//     Vector3f predict_magnetometer(const Quaternion &q) const;

//     void build_mag_measurement_jacobian(
//         const Vector3f &mag_prediction);

//     bool fuse_mag_component(const Vector3f &mag,
//                             uint8_t component);

//     // ---------------------------------------------------------------------
//     // Quaternion
//     // ---------------------------------------------------------------------
//     Quaternion get_state_quaternion() const;
//     void set_state_quaternion(const Quaternion &q);

//     Quaternion quaternion_from_gyro(const Vector3f &gyro,
//                                    float dt) const;

//     void propagate_quaternion(const Vector3f &gyro,
//                               float dt);

//     Matrix3f quaternion_to_rotation_matrix(
//         const Quaternion &q) const;

//     void normalize_quaternion();

//     // ---------------------------------------------------------------------
//     // Matrix helpers
//     // ---------------------------------------------------------------------
//     void zero_matrix(float A[STATE_SIZE][STATE_SIZE]);
//     void zero_matrix_x3(float A[STATE_SIZE][3]);
//     void zero_matrix_3x(float A[3][STATE_SIZE]);
//     void zero_matrix_3(float A[3][3]);
//     void identity_matrix(float A[STATE_SIZE][STATE_SIZE]);

//     bool inverse_3x3(const float A[3][3],
//                      float A_inv[3][3]) const;
// };


#pragma once

#include <stdint.h>
#include <math.h>

#include <AP_Math/AP_Math.h>

/*
 * OurEKF - custom 22 state EKF for CubeOrange Plus / ArduPilot (ChibiOS)
 *
 * Conventions
 *   - Quaternion state is body -> NED (same as ArduPilot).
 *   - quaternion_to_rotation_matrix() returns the NED -> body matrix (R'),
 *     so body -> NED is the transpose of it.
 *   - Specific force from the IMU, NED, gravity is +Z.
 */
class OurEKF
{
public:
    // ---------------------------------------------------------------------
    // State layout
    // ---------------------------------------------------------------------
    // 0..2   position NED [m]
    // 3..5   velocity NED [m/s]
    // 6..9   quaternion [qw qx qy qz]
    // 10..12 accelerometer bias [m/s^2]
    // 13..15 gyro bias [rad/s]
    // 16..18 earth magnetic field NED [sensor units]
    // 19..21 body magnetic bias/field [sensor units]
    // ---------------------------------------------------------------------
    static constexpr int STATE_SIZE = 22;
    static constexpr int GPS_SIZE   = 3;
    static constexpr int MAG_SIZE   = 3;

    OurEKF();

    void init(const Vector3f &position,
              const Vector3f &velocity,
              const Quaternion &quaternion);

    // Optional: seed the earth field (NED, same units as the compass, e.g.
    // from AP_Declination / WMM). Makes yaw absolute. Call after init().
    // If never called, the earth field is taken from the first compass sample
    // (yaw is then only relative to the initial quaternion).
    void init_earth_field(const Vector3f &earth_field_ned, float std_dev = 1.0f);

    // Helper: initial attitude from static accel (specific force) and mag.
    static Quaternion attitude_from_accel_mag(const Vector3f &accel,
                                              const Vector3f &mag,
                                              float declination_rad);

    void predict(const Vector3f &accel,
                 const Vector3f &gyro,
                 float dt);

    bool update_gps(const Vector3f &gps_position);
    bool update_gps_velocity(const Vector3f &gps_velocity);
    bool update_mag(const Vector3f &mag);

    // Original interface (position only GPS)
    void update(const Vector3f &accel,
                const Vector3f &gyro,
                const Vector3f &mag,
                float dt,
                bool gps_available,
                const Vector3f &gps_position);

    // Interface with GPS velocity
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
    // Body magnetic states are unobservable on the ground. Keep disabled
    // (default) until the vehicle is flying and manoeuvring.
    void set_body_mag_learning(bool enable);
    // Accelerometer gravity (tilt) aiding inside predict(). Default: on.
    void set_tilt_fusion(bool enable) { tilt_fusion_enabled = enable; }
    // Magnetometer noise std in the same units as the compass data.
    void set_mag_noise(float std_dev) { mag_noise_std = std_dev; }

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

    float get_innovation_x() const;
    float get_innovation_y() const;
    float get_innovation_z() const;

    Vector3f get_velocity_innovation() const;
    Vector3f get_mag_innovation() const;

    bool is_initialized() const;
    bool is_healthy() const { return healthy; }
    uint16_t get_reset_count() const { return reset_count; }

    // Last corrected accelerometer values, useful for debugging/logging.
    Vector3f last_accel_body;
    Vector3f last_accel_world;

private:
    enum FuseResult : uint8_t {
        FUSE_OK = 0,
        FUSE_REJECTED,   // failed innovation gate
        FUSE_INVALID     // numerically invalid innovation variance
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

    // Scalar fusion work: gain and P*H' (column 0 used).
    float K[STATE_SIZE][GPS_SIZE];
    float PHt[STATE_SIZE][GPS_SIZE];

    float innovation[GPS_SIZE];      // GPS position
    float innovation_vel[GPS_SIZE];  // GPS velocity
    float innovation_mag[MAG_SIZE];  // magnetometer

    // Persistent work buffers (members, not locals: ChibiOS task stack).
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

    bool mag_learning;          // body mag states enabled
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
    // Generic scalar fusion (sequential, O(n^2))
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

    // Returns NED -> body rotation matrix.
    Matrix3f quaternion_to_rotation_matrix(
        const Quaternion &q) const;

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
};

// #include "ownEkf.h"

// #include <string.h>
// #include <math.h>
// #include <AP_GPS/AP_GPS.h>

// #include <AP_HAL/AP_HAL.h>
// extern const AP_HAL::HAL& hal;
// //#include <AP_NavEKF/AP_NavEKF.h>


// // Constructor


// OurEKF::OurEKF()
// {
//     memset(x, 0, sizeof(x));
//     memset(P, 0, sizeof(P));
//     memset(F, 0, sizeof(F));
//     memset(Q, 0, sizeof(Q));
//     memset(H, 0, sizeof(H));
//     memset(R, 0, sizeof(R));
//     memset(S, 0, sizeof(S));
//     memset(K, 0, sizeof(K));
//     memset(innovation, 0, sizeof(innovation));

//     roll=0.0f;
//     pitch=0.0f;
//     yaw=35.11f;

//     initialized = false;

//     // calibration defaults
//     calib_active = false;
//     calib_target = 0;
//     calib_count = 0;
//     calib_sum_measured = Vector3f(0.0f, 0.0f, 0.0f);
//     calib_sum_expected = Vector3f(0.0f, 0.0f, 0.0f);
// }


// // Initialization


// void OurEKF::init(const Vector3f &position,
//                   const Vector3f &velocity,
//                   const Quaternion &quaternion)
// {
//     memset(x, 0, sizeof(x));
//     memset(P, 0, sizeof(P));

//     // Initial state


//     x[0] = position.x;
//     x[1] = position.y;
//     x[2] = position.z;

//     x[3] = velocity.x;
//     x[4] = velocity.y;
//     x[5] = velocity.z;

//     x[6] = quaternion.q1;
//     x[7] = quaternion.q2;
//     x[8] = quaternion.q3;
//     x[9] = quaternion.q4;

//     // Initial accelerometer bias
//     x[10] = 0.0f;
//     x[11] = 0.0f;
//     x[12] = 0.0f;

//     // Initial gyro bias
//     x[13] = 0.0f;
//     x[14] = 0.0f;
//     x[15] = 0.0f;

   
//     // Normalize quaternion
  

//     normalize_quaternion();

  
//     // Initial covariance
  
//     // These are example values.
//     // They MUST eventually be tuned for the actual system.
    
    

//     for (int i = 0; i < STATE_SIZE; i++) {
//         P[i][i] = 0.01f;
//     }

//     // Position uncertainty
//     P[0][0] = 1.0f;
//     P[1][1] = 1.0f;
//     P[2][2] = 1.0f;

//     // Velocity uncertainty
//     P[3][3] = 0.5f;
//     P[4][4] = 0.5f;
//     P[5][5] = 0.5f;

//     // Quaternion uncertainty
//     P[6][6] = 0.01f;
//     P[7][7] = 0.01f;
//     P[8][8] = 0.01f;
//     P[9][9] = 0.01f;

//     // Accelerometer bias uncertainty
//     P[10][10] = 0.01f;
//     P[11][11] = 0.01f;
//     P[12][12] = 0.01f;

//     // Gyro bias uncertainty
//     P[13][13] = 0.01f;
//     P[14][14] = 0.01f;
//     P[15][15] = 0.01f;

//     initialized = true;
// }



// // Complete EKF update


// void OurEKF::update(const Vector3f &accel,
//                     const Vector3f &gyro,
//                     const Vector3f &mag,
//                     float dt,
//                     bool gps_available,
//                     const Vector3f &gps_position)
// {
//     if (!initialized) {
//         return;
//     }  

//     // IMU Prediction
//     predict(accel, gyro, dt); 

//     // GPS Correction
//     if (gps_available) {
//         update_gps(gps_position);
//     }

//     // Magnetometer Correction
//     //update_mag(mag);
// }
// // Prediction
// void OurEKF::predict(const Vector3f &accel,
//                      const Vector3f &gyro,
//                      float dt)
// {
//     if (!initialized) {
//         return;
//     }

//     if (dt <= 0.0f) {
//         return;
//     }


//     // STAGE 2
//     // Remove estimated sensor biases
 

//     Vector3f accel_corrected;
//     Vector3f gyro_corrected;

//     accel_corrected.x = accel.x - x[10];
//     accel_corrected.y = accel.y - x[11];
//     accel_corrected.z = accel.z - x[12];

//     gyro_corrected.x = gyro.x - x[13];
//     gyro_corrected.y = gyro.y - x[14];
//     gyro_corrected.z = gyro.z - x[15];

//     last_accel_body = accel_corrected;
    
//     // STAGE 3
//     // Quaternion prediction
   

//     propagate_quaternion(gyro_corrected, dt);


  
//     // STAGE 4
//     // Body frame -> world frame
    

//     Quaternion q = get_state_quaternion();

//     Matrix3f R_body_to_world =
//         quaternion_to_rotation_matrix(q);

//     Vector3f accel_world;

//     accel_world.x =
//         R_body_to_world.a.x * accel_corrected.x +
//         R_body_to_world.b.x * accel_corrected.y +
//         R_body_to_world.c.x * accel_corrected.z;

//     accel_world.y =
//         R_body_to_world.a.y * accel_corrected.x +
//         R_body_to_world.b.y * accel_corrected.y +
//         R_body_to_world.c.y * accel_corrected.z;

//     accel_world.z =
//         R_body_to_world.a.z * accel_corrected.x +
//         R_body_to_world.b.z * accel_corrected.y +
//         R_body_to_world.c.z * accel_corrected.z;


//     // STAGE 5
//     // Gravity compensation
    
//     // Current convention:
//     // world +Z = up
//     // gravity = [0,0,-9.81]
    
//     // Specific-force convention:
    
//     // linear acceleration =
//     //       rotated acceleration + [0,0,9.81]
    

//     accel_world.z += 9.81f;

//     last_accel_world = accel_world; 


    
//     // STAGE 7
//     // Position prediction
    
//     // p = p + v*dt + 0.5*a*dt^2
   

//     float half_dt2 = 0.5f * dt * dt;

//     x[0] += x[3] * dt + accel_world.x * half_dt2;
//     x[1] += x[4] * dt + accel_world.y * half_dt2;
//     x[2] += x[5] * dt + accel_world.z * half_dt2;


    
//     // STAGE 6
//     // Velocity prediction
    
//     // v = v + a*dt
//     x[3] += accel_world.x * dt;
//     x[4] += accel_world.y * dt;
//     x[5] += accel_world.z * dt;


//     // STAGE 8
//     // Bias prediction
    
//     // Biases are random walk states.
    
//     // b(k) = b(k-1) + w
    
//     // Mean prediction therefore keeps the current value.
    


//     // STAGE 9
//     // Build F
   

//     build_F(accel_corrected, dt);


//     // STAGE 10
//     // Build Q
    

//     build_Q(dt);



//     // STAGE 11
//     // Covariance prediction
    
//     // P = F P F^T + Q
  

//     predict_covariance();


   
//     // Quaternion normalization
   

//     normalize_quaternion();

//     Orientation();

//     hal.console->printf("GYRO: %.4f %.4f %.4f\n",
//     gyro.x, gyro.y, gyro.z);

//     hal.console->printf("ACCEL: %.4f %.4f %.4f\n",
//     accel.x, accel.y, accel.z);

//     hal.console->printf("DT: %.6f\n", dt);

//     hal.console->printf("Q: %.4f %.4f %.4f %.4f\n",
//     x[6], x[7], x[8], x[9]);

//     hal.console->printf("RPY: %.2f %.2f %.2f\n",
//     get_roll()  * RAD_TO_DEG,
//     get_pitch() * RAD_TO_DEG,
//     get_yaw()   * RAD_TO_DEG);


//     hal.console->printf("GPS K quat:\n");

//     hal.console->printf("Kq0: %.6f %.6f %.6f\n",
//     K[6][0], K[6][1], K[6][2]);

//     hal.console->printf("Kq1: %.6f %.6f %.6f\n",
//     K[7][0], K[7][1], K[7][2]);

//     hal.console->printf("Kq2: %.6f %.6f %.6f\n",
//     K[8][0], K[8][1], K[8][2]);

//     hal.console->printf("Kq3: %.6f %.6f %.6f\n",
//        K[9][0], K[9][1], K[9][2]);
// }



// // Quaternion prediction


// void OurEKF::propagate_quaternion(const Vector3f &gyro,
//                                   float dt)
// {
//     Quaternion q = get_state_quaternion();

//     Quaternion dq = quaternion_from_gyro(gyro, dt);

//     // Quaternion multiplication
//     Quaternion q_new;

//     q_new.q1 =
//         q.q1 * dq.q1 -
//         q.q2 * dq.q2 -
//         q.q3 * dq.q3 -
//         q.q4 * dq.q4;

//     q_new.q2 =
//         q.q1 * dq.q2 +
//         q.q2 * dq.q1 +
//         q.q3 * dq.q4 -
//         q.q4 * dq.q3;

//     q_new.q3 =
//         q.q1 * dq.q3 -
//         q.q2 * dq.q4 +
//         q.q3 * dq.q1 +
//         q.q4 * dq.q2;

//     q_new.q4 =
//         q.q1 * dq.q4 +
//         q.q2 * dq.q3 -
//         q.q3 * dq.q2 +
//         q.q4 * dq.q1;

//     set_state_quaternion(q_new);

//     normalize_quaternion();
// }

// void OurEKF::Orientation()
// {
//     Quaternion q = get_state_quaternion();

//     float qw = q.q1;
//     float qx = q.q2;
//     float qy = q.q3;
//     float qz = q.q4;

//     // Roll
//     roll = atan2f(
//         2.0f * (qw * qx + qy * qz),
//         1.0f - 2.0f * (qx * qx + qy * qy)
//     );

//     // Pitch
//     float sin_pitch =
//         2.0f * (qw * qy - qz * qx);

//     // protect against floating point values slightly > 1 or < -1
//     if (sin_pitch > 1.0f) {
//         sin_pitch = 1.0f;
//     }

//     if (sin_pitch < -1.0f) {
//         sin_pitch = -1.0f;
//     }

//     pitch = asinf(sin_pitch);

//     // Yaw
//     yaw = atan2f(
//         2.0f * (qw * qz + qx * qy),
//         1.0f - 2.0f * (qy * qy + qz * qz)
//     );
// }



// // Create delta quaternion from gyro


// Quaternion OurEKF::quaternion_from_gyro(const Vector3f &gyro,
//                                          float dt) const
// {
//     float angle_x = gyro.x * dt;
//     float angle_y = gyro.y * dt;
//     float angle_z = gyro.z * dt;

//     float angle =
//         sqrtf(angle_x * angle_x +
//               angle_y * angle_y +
//               angle_z * angle_z);

//     Quaternion dq;

//     if (angle < 1.0e-8f) {

//         dq.q1 = 1.0f;
//         dq.q2 = 0.0f;
//         dq.q3 = 0.0f;
//         dq.q4 = 0.0f;

//         return dq;
//     }

//     float half_angle = 0.5f * angle;

//     float s = sinf(half_angle) / angle;

//     dq.q1 = cosf(half_angle);
//     dq.q2 = angle_x * s;
//     dq.q3 = angle_y * s;
//     dq.q4 = angle_z * s;

//     return dq;
// }


// // Quaternion -> rotation matrix


// Matrix3f OurEKF::quaternion_to_rotation_matrix(
//     const Quaternion &q) const
// {
//     Matrix3f Root;

//     float qw = q.q1;
//     float qx = q.q2;
//     float qy = q.q3;
//     float qz = q.q4;

//     Root.a.x = 1.0f - 2.0f * (qy * qy + qz * qz);
//     Root.a.y = 2.0f * (qx * qy + qz * qw);
//     Root.a.z = 2.0f * (qx * qz - qy * qw);

//     Root.b.x = 2.0f * (qx * qy - qz * qw);
//     Root.b.y = 1.0f - 2.0f * (qx * qx + qz * qz);
//     Root.b.z = 2.0f * (qy * qz + qx * qw);

//     Root.c.x = 2.0f * (qx * qz + qy * qw);
//     Root.c.y = 2.0f * (qy * qz - qx * qw);
//     Root.c.z = 1.0f - 2.0f * (qx * qx + qy * qy);

//     return Root;
// }


// // Build F


// void OurEKF::build_F(const Vector3f &accel_body,
//                      float dt)
// {
//     zero_matrix_16(F);

//     // Identity blocks
    

//     F[0][0] = 1.0f;
//     F[1][1] = 1.0f;
//     F[2][2] = 1.0f;

//     F[3][3] = 1.0f;
//     F[4][4] = 1.0f;
//     F[5][5] = 1.0f;

//     F[6][6] = 1.0f;
//     F[7][7] = 1.0f;
//     F[8][8] = 1.0f;
//     F[9][9] = 1.0f;

//     F[10][10] = 1.0f;
//     F[11][11] = 1.0f;
//     F[12][12] = 1.0f;

//     F[13][13] = 1.0f;
//     F[14][14] = 1.0f;
//     F[15][15] = 1.0f;

//     // Position <- velocity
    
//     // p(k+1) = p(k) + v(k)dt + ...
   

//     F[0][3] = dt;
//     F[1][4] = dt;
//     F[2][5] = dt;


    
//     // Position <- accelerometer bias
    
//     // approximately:
    
//     // F_p,ba = -0.5 R dt²


//     Quaternion q = get_state_quaternion();

//     Matrix3f Rot = quaternion_to_rotation_matrix(q);


//     float half_dt2 = 0.5f * dt * dt;

//     F[0][10] = -half_dt2 * Rot.a.x;
//     F[0][11] = -half_dt2 * Rot.b.x;
//     F[0][12] = -half_dt2 * Rot.c.x;

//     F[1][10] = -half_dt2 * Rot.a.y;
//     F[1][11] = -half_dt2 * Rot.b.y;
//     F[1][12] = -half_dt2 * Rot.c.y;

//     F[2][10] = -half_dt2 * Rot.a.z;
//     F[2][11] = -half_dt2 * Rot.b.z;
//     F[2][12] = -half_dt2 * Rot.c.z;


//     // Velocity <- accelerometer bias
   
//     // F_v,ba = -R dt
  

//     F[3][10] = -dt * Rot.a.x;
//     F[3][11] = -dt * Rot.b.x;
//     F[3][12] = -dt * Rot.c.x;

//     F[4][10] = -dt * Rot.a.y;
//     F[4][11] = -dt * Rot.b.y;
//     F[4][12] = -dt * Rot.c.y;

//     F[5][10] = -dt * Rot.a.z;
//     F[5][11] = -dt * Rot.b.z;
//     F[5][12] = -dt * Rot.c.z;


//     // Quaternion / gyro-bias relationships
    
//     // A first-order approximation is used here.
    
//     // q(k+1) ≈ q(k) + 0.5 Ω(q) ω dt
    
//     // Therefore:
    
//     // F_q,q ≈ I + 0.5 Ω(ω)dt
    
//     // F_q,bg ≈ -0.5 Ω(q)dt


//     Vector3f gyro;

//     // We do not have the raw gyro inside this function.
//     // Use zero angular rate for the linearized quaternion block.
//     gyro.x = 0.0f;
//     gyro.y = 0.0f;
//     gyro.z = 0.0f;

//     float half_dt = 0.5f * dt;

//     F[6][6] = 1.0f;

//     F[7][7] = 1.0f;
//     F[8][8] = 1.0f;
//     F[9][9] = 1.0f;

  
//     // qdot = 0.5 * q ⊗ [0,omega]
   

//     float qw = q.q1;
//     float qx = q.q2;
//     float qy = q.q3;
//     float qz = q.q4;

//     F[6][13] =  half_dt * qx;
//     F[6][14] =  half_dt * qy;
//     F[6][15] =  half_dt * qz;

//     F[7][13] = -half_dt * qw;
//     F[7][14] =  half_dt * qz;
//     F[7][15] = -half_dt * qy;

//     F[8][13] = -half_dt * qz;
//     F[8][14] = -half_dt * qw;
//     F[8][15] =  half_dt * qx;

//     F[9][13] =  half_dt * qy;
//     F[9][14] = -half_dt * qx;
//     F[9][15] = -half_dt * qw;


    
//     // Approximate attitude -> acceleration coupling.
    
//     // For the initial implementation this is intentionally simplified.
//     // A full analytical J_q should be derived consistently with the exact
//     // quaternion convention before using this EKF for serious navigation.
   

//     (void)accel_body;
// }



// // Build Q


// void OurEKF::build_Q(float dt)
// {
//     zero_matrix_16(Q);


//     // Example sensor noise parameters.
    
//     // These are NOT universal values.
//     // They must be obtained/tuned from the actual IMU.
    

//     const float accel_noise_std = 0.20f;
//     const float gyro_noise_std  = 0.02f;

//     const float accel_bias_rw_std = 0.001f;
//     const float gyro_bias_rw_std  = 0.0005f;

//     const float accel_var =
//         accel_noise_std * accel_noise_std;

//     const float gyro_var =
//         gyro_noise_std * gyro_noise_std;

//     const float accel_bias_var =
//         accel_bias_rw_std * accel_bias_rw_std;

//     const float gyro_bias_var =
//         gyro_bias_rw_std * gyro_bias_rw_std;


    
//     // Position process noise
    
//     // Q_p ≈ 1/4 R Sigma_a R^T dt^4
    
//     // Simplified here as diagonal.
    
//     float dt2 = dt * dt;
//     float dt4 = dt2 * dt2;

//     float position_noise =
//         0.25f * accel_var * dt4;

//     Q[0][0] = position_noise;
//     Q[1][1] = position_noise;
//     Q[2][2] = position_noise;


//     // Velocity process noise
    
//     // Q_v ≈ R Sigma_a R^T dt²
    
//     // Simplified diagonal.
   

//     float velocity_noise =
//         accel_var * dt2;

//     Q[3][3] = velocity_noise;
//     Q[4][4] = velocity_noise;
//     Q[5][5] = velocity_noise;


   
//     // Quaternion process noise
  
//     float quaternion_noise =
//         0.25f * gyro_var * dt2;

//     Q[6][6] = quaternion_noise;
//     Q[7][7] = quaternion_noise;
//     Q[8][8] = quaternion_noise;
//     Q[9][9] = quaternion_noise;


   
//     // Accelerometer bias random walk
  
//     Q[10][10] = accel_bias_var * dt;
//     Q[11][11] = accel_bias_var * dt;
//     Q[12][12] = accel_bias_var * dt;


    
//     // Gyro bias random walk
    
//     Q[13][13] = gyro_bias_var * dt;
//     Q[14][14] = gyro_bias_var * dt;
//     Q[15][15] = gyro_bias_var * dt;
// }



// // Covariance prediction

// // P = F P F^T + Q


// void OurEKF::predict_covariance()
// {
//     float temp[STATE_SIZE][STATE_SIZE] = {};

    
//     // temp = F * P
    
//     for (int i = 0; i < STATE_SIZE; i++) {
//         for (int j = 0; j < STATE_SIZE; j++) {

//             float sum = 0.0f;

//             for (int k = 0; k < STATE_SIZE; k++) {
//                 sum += F[i][k] * P[k][j];
//             }

//             temp[i][j] = sum;
//         }
//     }


//     // P_new = temp * F^T + Q
   
//     for (int i = 0; i < STATE_SIZE; i++) {
//         for (int j = 0; j < STATE_SIZE; j++) {

//             float sum = 0.0f;

//             for (int k = 0; k < STATE_SIZE; k++) {
//                 sum += temp[i][k] * F[j][k];
//             }

//             P[i][j] = sum + Q[i][j];
//         }
//     }
// }


// // GPS UPDATE


// bool OurEKF::update_gps(const Vector3f &gps_position)
// {
//     int i=0, j=0;
//     if (!initialized) {
//         return false;
//     }


  
//     // STAGE 13 / 14
//     // Measurement model and H


//     build_H();



//     // STAGE 15
//     // Predicted measurement
    

//     float z_pred[3];

//     z_pred[0] = x[0];
//     z_pred[1] = x[1];
//     z_pred[2] = x[2];


    
//     // STAGE 16
//     // Innovation
    
//     // y = z - z_hat
    

//     innovation[0] =
//         gps_position.x - z_pred[0];

//     innovation[1] =
//         gps_position.y - z_pred[1];

//     innovation[2] =
//         gps_position.z - z_pred[2];


   
//     // STAGE 17
//     // Measurement covariance R
    
//     build_R();


    
//     // STAGE 18
//     // Innovation covariance S
    
//     // S = H P H^T + R
   

//     if (!calculate_innovation_covariance()) {
//         return false;
//     }


   
//     // STAGE 19
//     // Kalman gain
    
//     // K = P H^T S^-1
    
//     if (!calculate_kalman_gain()) {
//         return false;
//     }


   
//     // STAGE 20
//     // State correction
    
//     // x = x + K y
    

//     correct_state();


    
//     // STAGE 21
//     // Covariance correction
   

//     correct_covariance();


    
//     // STAGE 22
//     // Quaternion normalization
    

//     normalize_quaternion();

//     x[i] += K[i][j] * innovation[j];

//     return true;
// }




// // bool OurEKF::update_mag(const Vector3f &mag)
// // {
// //     if (!initialized) {
// //         return false;
// //     }

    
// //     // 1. Calculate magnetic heading from existing AP_Compass data
   
// //     float mag_yaw = atan2f(-mag.y, mag.x);

   
// //     // 2. Get current EKF yaw from quaternion
    
// //     Quaternion q = get_state_quaternion();

// //     float ekf_yaw = atan2f(
// //         2.0f * (q.q1 * q.q4 + q.q2 * q.q3),
// //         1.0f - 2.0f * (q.q3 * q.q3 + q.q4 * q.q4)
// //     );

   
// //     // 3. Calculate yaw innovation
   

// //     float yaw_error = mag_yaw - ekf_yaw;

// //     // Wrap to -PI ... +PI
// //     while (yaw_error > M_PI) {
// //         yaw_error -= 2.0f * M_PI;
// //     }

// //     while (yaw_error < -M_PI) {
// //         yaw_error += 2.0f * M_PI;
// //     }

// //     innovation_mag[0] = yaw_error;

   
// //     // 4. Measurement Jacobian
    
// //     // For this simple implementation we treat yaw as the
// //     // measurement and use a simple sensitivity to the
// //     // quaternion attitude states.
    

// //     // for (int i = 0; i < STATE_SIZE; i++) {
// //     //     H_mag[0][i] = 0.0f;
// //     // }

// //     // // Numerical/simple yaw sensitivity.
  
// //     // // This is intentionally simple for your first implementation.
// //     // H_mag[0][8] = 1.0f;
// //     // H_mag[0][9] = 1.0f;



// //     float qw = q.q1;
// //     float qx = q.q2;
// //     float qy = q.q3;
// //     float qz = q.q4;

// //     float a = 2.0f * (qw*qz + qx*qy);
// //     float b = 1.0f - 2.0f * (qy*qy + qz*qz);

// //     float denom = a*a + b*b;
    
// //     for (int i = 0; i < STATE_SIZE; i++) {
// //     H_mag[0][i] = 0.0f;
// //     }


// //     H_mag[0][6] = (b * (2.0f*qz)) / denom;                       // d yaw / d qw

// //     H_mag[0][7] = (b * (2.0f*qy)) / denom;                       // d yaw / d qx

// //     H_mag[0][8] = (b * (2.0f*qx) - a * (-4.0f*qy)) / denom;     // d yaw / d qy

// //     H_mag[0][9] = (b * (2.0f*qw) - a * (-4.0f*qz)) / denom;     // d yaw / d qz

// //     // 5. Measurement noise
   

// //     R_mag[0][0] = sq(5.0f * DEG_TO_RAD);

   
// //     // 6. Innovation covariance

// //     // S = HPH' + R


// //     float S_value = R_mag[0][0];

// //     for (int i = 0; i < STATE_SIZE; i++) {
// //         for (int j = 0; j < STATE_SIZE; j++) {
// //             S_value += H_mag[0][i] *
// //                        P[i][j] *
// //                        H_mag[0][j];
// //         }
// //     }

// //     if (S_value <= 0.0f) {
// //         return false;
// //     }

// //     S_mag[0][0] = S_value;

   
// //     // 7. Kalman gain
   
// //     // K = P H' / S
   
// //     for (int i = 0; i < STATE_SIZE; i++) {

// //         float value = 0.0f;

// //         for (int j = 0; j < STATE_SIZE; j++) {
// //             value += P[i][j] * H_mag[0][j];
// //         }

// //         K_mag[i][0] = value / S_value;
// //     }

   
// //     // 8. Correct state
  
// //     // x = x + K * innovation
   
// //     for (int i = 0; i < STATE_SIZE; i++) {
// //         x[i] += K_mag[i][0] * innovation_mag[0];
// //     }

  
// //     // 9. Normalize quaternion
    

// //     normalize_quaternion();

    
// //     // 10. Correct covariance
    
// //     // P = (I - KH)P
   

// //     float P_new[STATE_SIZE][STATE_SIZE];

// //     for (int i = 0; i < STATE_SIZE; i++) {

// //         for (int j = 0; j < STATE_SIZE; j++) {

// //             float value = P[i][j];

// //             for (int k = 0; k < STATE_SIZE; k++) {
// //                 value -= K_mag[i][0] *
// //                          H_mag[0][k] *
// //                          P[k][j];
// //             }

// //             P_new[i][j] = value;
// //         }
// //     }

// //     for (int i = 0; i < STATE_SIZE; i++) {
// //         for (int j = 0; j < STATE_SIZE; j++) {
// //             P[i][j] = P_new[i][j];
// //         }
// //     }

// //     return true;
// // }

// // bool OurEKF::update_mag(const Vector3f &mag)
// // {
// //     if (!initialized) {
// //         return false;
// //     }

// //     // ---------------------------------------------------------
// //     // 1. Calculate magnetic heading
// //     // ---------------------------------------------------------

// //     float mag_yaw = atan2f(-mag.y, mag.x);

// //     // ---------------------------------------------------------
// //     // 2. Get current EKF quaternion
// //     // ---------------------------------------------------------

// //     Quaternion q = get_state_quaternion();

// //     float qw = q.q1;
// //     float qx = q.q2;
// //     float qy = q.q3;
// //     float qz = q.q4;

// //     // ---------------------------------------------------------
// //     // 3. Calculate current EKF yaw
// //     // ---------------------------------------------------------

// //     float a = 2.0f * (qw*qz + qx*qy);

// //     float b = 1.0f - 2.0f * (qy*qy + qz*qz);

// //     float ekf_yaw = atan2f(a, b);

    
// //     // 4. Innovation
    

// //     float yaw_error = mag_yaw - ekf_yaw;

// //     while (yaw_error > M_PI) {
// //         yaw_error -= 2.0f * M_PI;
// //     }

// //     while (yaw_error < -M_PI) {
// //         yaw_error += 2.0f * M_PI;
// //     }

// //     innovation_mag[0] = yaw_error;

// //     // ---------------------------------------------------------
// //     // 5. Build H
// //     // ---------------------------------------------------------

// //     for (int i = 0; i < STATE_SIZE; i++) {
// //         H_mag[0][i] = 0.0f;
// //     }

// //     float denom = a*a + b*b;

// //     if (denom < 1.0e-6f) {
// //         return false;
// //     }


// //     // dN/dq
// //     float dN_dqw = 2.0f * qz;
// //     float dN_dqx = 2.0f * qy;
// //     float dN_dqy = 2.0f * qx;
// //     float dN_dqz = 2.0f * qw;

// //     // dD/dq
// //     float dD_dqw = 0.0f;
// //     float dD_dqx = 0.0f;
// //     float dD_dqy = -4.0f * qy;
// //     float dD_dqz = -4.0f * qz;




// //     // d(yaw)/d(qw)
// //     H_mag[0][6] =
// //         (b * (2.0f*qz)) / denom;

// //     // d(yaw)/d(qx)
// //     H_mag[0][7] =
// //         (b * (2.0f*qy)) / denom;

// //     // d(yaw)/d(qy)
// //     H_mag[0][8] =
// //         (b * (2.0f*qx) -
// //          a * (-4.0f*qy)) / denom;

// //     // d(yaw)/d(qz)
// //     H_mag[0][9] =
// //         (b * (2.0f*qw) -
// //          a * (-4.0f*qz)) / denom;

// //     // ---------------------------------------------------------
// //     // 6. Measurement noise
// //     // ---------------------------------------------------------

// //     R_mag[0][0] = sq(5.0f * DEG_TO_RAD);

// //     // ---------------------------------------------------------
// //     // 7. S = HPH' + R
// //     // ---------------------------------------------------------

// //     float S_value = R_mag[0][0];

// //     for (int i = 0; i < STATE_SIZE; i++) {
// //         for (int j = 0; j < STATE_SIZE; j++) {

// //             S_value +=
// //                 H_mag[0][i] *
// //                 P[i][j] *
// //                 H_mag[0][j];
// //         }
// //     }

// //     if (S_value <= 1.0e-9f) {
// //         return false;
// //     }

// //     // ---------------------------------------------------------
// //     // 8. K = PH'/S
// //     // ---------------------------------------------------------

// //     for (int i = 0; i < STATE_SIZE; i++) {

// //         float value = 0.0f;

// //         for (int j = 0; j < STATE_SIZE; j++) {

// //             value +=
// //                 P[i][j] *
// //                 H_mag[0][j];
// //         }

// //         K_mag[i][0] = value / S_value;
// //     }

// //     // ---------------------------------------------------------
// //     // 9. State correction
// //     // ---------------------------------------------------------

// //     for (int i = 0; i < STATE_SIZE; i++) {

// //         x[i] +=
// //             K_mag[i][0] *
// //             innovation_mag[0];
// //     }

// //     // ---------------------------------------------------------
// //     // 10. Normalize quaternion
// //     // ---------------------------------------------------------

// //     normalize_quaternion();

// //     // ---------------------------------------------------------
// //     // 11. Covariance correction
// //     // ---------------------------------------------------------

// //     float P_new[STATE_SIZE][STATE_SIZE];

// //     for (int i = 0; i < STATE_SIZE; i++) {

// //         for (int j = 0; j < STATE_SIZE; j++) {

// //             float value = P[i][j];

// //             for (int k = 0; k < STATE_SIZE; k++) {

// //                 value -=
// //                     K_mag[i][0] *
// //                     H_mag[0][k] *
// //                     P[k][j];
// //             }

// //             P_new[i][j] = value;
// //         }
// //     }

// //     for (int i = 0; i < STATE_SIZE; i++) {
// //         for (int j = 0; j < STATE_SIZE; j++) {
// //             P[i][j] = P_new[i][j];
// //         }
// //     }

// //     return true;
// // }


// bool OurEKF::update_mag(const Vector3f &mag)
// {
//     if (!initialized) {
//         return false;
//     }

//     // ---------------------------------------------------------
//     // 1. Calculate magnetic heading
//     // ---------------------------------------------------------

//     float mag_yaw = atan2f(-mag.y, mag.x);

//     // ---------------------------------------------------------
//     // 2. Get current EKF quaternion
//     // ---------------------------------------------------------

//     Quaternion q = get_state_quaternion();

//     float qw = q.q1;
//     float qx = q.q2;
//     float qy = q.q3;
//     float qz = q.q4;

//     // ---------------------------------------------------------
//     // 3. Calculate current EKF yaw
//     //
//     // yaw = atan2(a, b)
//     //
//     // a = 2(qw*qz + qx*qy)
//     // b = 1 - 2(qy^2 + qz^2)
//     // ---------------------------------------------------------

//     float a = 2.0f * (qw*qz + qx*qy);

//     float b = 1.0f - 2.0f * (qy*qy + qz*qz);

//     float ekf_yaw = atan2f(a, b);

//     // ---------------------------------------------------------
//     // 4. Calculate yaw innovation
//     // ---------------------------------------------------------

//     float yaw_error = mag_yaw - ekf_yaw;

//     // Wrap innovation to [-PI, +PI]
//     while (yaw_error > M_PI) {
//         yaw_error -= 2.0f * M_PI;
//     }

//     while (yaw_error < -M_PI) {
//         yaw_error += 2.0f * M_PI;
//     }

//     innovation_mag[0] = yaw_error;

//     // ---------------------------------------------------------
//     // 5. Build measurement Jacobian H
//     //
//     // Measurement:
//     //
//     // z = yaw
//     //
//     // H = d(yaw)/d(state)
//     //
//     // Only quaternion states are involved:
//     //
//     // x[6] = qw
//     // x[7] = qx
//     // x[8] = qy
//     // x[9] = qz
//     // ---------------------------------------------------------

//     for (int i = 0; i < STATE_SIZE; i++) {
//         H_mag[0][i] = 0.0f;
//     }

//     float denom = a*a + b*b;

//     if (denom < 1.0e-6f) {
//         return false;
//     }

//     // ---------------------------------------------------------
//     // Derivatives of a
//     //
//     // a = 2(qw*qz + qx*qy)
//     // ---------------------------------------------------------

//     float da_dqw = 2.0f * qz;
//     float da_dqx = 2.0f * qy;
//     float da_dqy = 2.0f * qx;
//     float da_dqz = 2.0f * qw;

//     // ---------------------------------------------------------
//     // Derivatives of b
//     //
//     // b = 1 - 2(qy^2 + qz^2)
//     // ---------------------------------------------------------

//     float db_dqw = 0.0f;
//     float db_dqx = 0.0f;
//     float db_dqy = -4.0f * qy;
//     float db_dqz = -4.0f * qz;

//     // ---------------------------------------------------------
//     // d atan2(a,b) / dq
//     //
//     // dyaw/dq =
//     //
//     // (b * da/dq - a * db/dq)
//     // ------------------------
//     //       a^2 + b^2
//     // ---------------------------------------------------------

//     H_mag[0][6] =
//         (b * da_dqw - a * db_dqw) / denom;

//     H_mag[0][7] =
//         (b * da_dqx - a * db_dqx) / denom;

//     H_mag[0][8] =
//         (b * da_dqy - a * db_dqy) / denom;

//     H_mag[0][9] =
//         (b * da_dqz - a * db_dqz) / denom;

//     // ---------------------------------------------------------
//     // 6. Magnetometer measurement noise
//     //
//     // 5 degrees standard deviation
//     // ---------------------------------------------------------

//     R_mag[0][0] =
//         sq(5.0f * DEG_TO_RAD);

//     // ---------------------------------------------------------
//     // 7. Innovation covariance
//     //
//     // S = H P H' + R
//     // ---------------------------------------------------------

//     float S_value = R_mag[0][0];

//     for (int i = 0; i < STATE_SIZE; i++) {

//         for (int j = 0; j < STATE_SIZE; j++) {

//             S_value +=
//                 H_mag[0][i] *
//                 P[i][j] *
//                 H_mag[0][j];
//         }
//     }

//     if (S_value <= 1.0e-9f) {
//         return false;
//     }

//     S_mag[0][0] = S_value;

//     // ---------------------------------------------------------
//     // 8. Kalman gain
//     //
//     // K = P H' / S
//     // ---------------------------------------------------------

//     for (int i = 0; i < STATE_SIZE; i++) {

//         float value = 0.0f;

//         for (int j = 0; j < STATE_SIZE; j++) {

//             value +=
//                 P[i][j] *
//                 H_mag[0][j];
//         }

//         K_mag[i][0] = value / S_value;
//     }

//     // ---------------------------------------------------------
//     // 9. Correct state
//     //
//     // x = x + K * innovation
//     // ---------------------------------------------------------

//     for (int i = 0; i < STATE_SIZE; i++) {

//         x[i] +=
//             K_mag[i][0] *
//             innovation_mag[0];
//     }

//     // ---------------------------------------------------------
//     // 10. Normalize quaternion
//     // ---------------------------------------------------------

//     normalize_quaternion();

//     // ---------------------------------------------------------
//     // 11. Correct covariance
//     //
//     // P = (I - K H) P
//     // ---------------------------------------------------------

//     float P_new[STATE_SIZE][STATE_SIZE];

//     for (int i = 0; i < STATE_SIZE; i++) {

//         for (int j = 0; j < STATE_SIZE; j++) {

//             float value = P[i][j];

//             for (int k = 0; k < STATE_SIZE; k++) {

//                 value -=
//                     K_mag[i][0] *
//                     H_mag[0][k] *
//                     P[k][j];
//             }

//             P_new[i][j] = value;
//         }
//     }

//     // Copy corrected covariance
//     for (int i = 0; i < STATE_SIZE; i++) {

//         for (int j = 0; j < STATE_SIZE; j++) {

//             P[i][j] = P_new[i][j];
//         }
//     }

//     hal.console->printf(
//     "MAG yaw=%.2f EKF yaw=%.2f error=%.2f\n",
//     mag_yaw * RAD_TO_DEG,
//     ekf_yaw * RAD_TO_DEG,
//     yaw_error * RAD_TO_DEG
//     );

//     return true;
// }


// // H

// // GPS directly measures:

// // z = [px py pz]

// // Therefore:

// // H =
// // [1 0 0 ...]
// // [0 1 0 ...]
// // [0 0 1 ...]

// //Measurement Model tells which state varible influences this measurement

// //after kalman gain, the row related to bax will have non-zero, so bias can be updated.



// void OurEKF::build_H()
// {
//     zero_matrix_3x16(H);

//     H[0][0] = 1.0f;
//     H[1][1] = 1.0f;
//     H[2][2] = 1.0f;
// }



// // R measurement covariance or uncertainity 

// void OurEKF::build_R()
// {
//     zero_matrix_3(R);

//     // Example GPS standard deviations.
//     // Units: metres.

//     const float gps_x_std = 2.0f;
//     const float gps_y_std = 2.0f;
//     const float gps_z_std = 4.0f;

//     R[0][0] =
//         gps_x_std * gps_x_std;

//     R[1][1] =
//         gps_y_std * gps_y_std;

//     R[2][2] =
//         gps_z_std * gps_z_std;
// }



// // Calculate S

// // S = H P H^T + R

// // Since H selects position:

// // S =
// // [P00 P01 P02]
// // [P10 P11 P12]
// // [P20 P21 P22]
// //
// // + R



// // S = HPH^T + R

// bool OurEKF::calculate_innovation_covariance()
// {
//     zero_matrix_3(S);

//     for (int i = 0; i < GPS_SIZE; i++) {

//         for (int j = 0; j < GPS_SIZE; j++) {

//             for (int k = 0; k < STATE_SIZE; k++) {

//                 for (int l = 0; l < STATE_SIZE; l++) {

//                     S[i][j] += H[i][k] * P[k][l] * H[j][l];
//                 }
//             }

//           S[i][j] += R[i][j];  
//         }
        
//     }

//     return true;
// }



// // Calculate Kalman gain

// // K = P*H^T*S^-1

// //decides whom to believe more, the prediction or the measurement.

// bool OurEKF::calculate_kalman_gain()
// {
//     float S_inv[3][3] = {};

//     if (!inverse_3x3(S, S_inv)) {
//         return false;
//     }


   
//     // PH^T
    
//     float PHt[STATE_SIZE][GPS_SIZE] = {};

//     for (int i = 0; i < STATE_SIZE; i++) {

//         for (int j = 0; j < GPS_SIZE; j++) {

//             for (int k = 0; k < STATE_SIZE; k++) {

//                 PHt[i][j] +=
//                     P[i][k] *
//                     H[j][k];
//             }
//         }
//     }


   
//     // K = PH^T S^-1
  
//     for (int i = 0; i < STATE_SIZE; i++) {

//         for (int j = 0; j < GPS_SIZE; j++) {

//             K[i][j] = 0.0f;

//             for (int k = 0; k < GPS_SIZE; k++) {

//                 K[i][j] +=
//                     PHt[i][k] *
//                     S_inv[k][j];
//             }
//         }
//     }

//     return true;
// }



// // Correct state

// // x = x + K innovation

// // x = K + Innovation

// void OurEKF::correct_state()
// {
//     float dx[STATE_SIZE] = {};

//     for (int i = 0; i < STATE_SIZE; i++) {

//         for (int j = 0; j < GPS_SIZE; j++) {

//             dx[i] +=
//                 K[i][j] *
//                 innovation[j];
//         }
//     }


//     for (int i = 0; i < STATE_SIZE; i++) {

//         x[i] += dx[i];
//     }
// }



// // Correct covariance

// // Joseph form:

// // P = (I-KH) P (I-KH)^T + K R K^T

// // This is numerically safer than simply:

// // P = (I-KH)P


// void OurEKF::correct_covariance()
// {
   

    
//     // temp = (I - K*H)
   
//     for (int i = 0; i < STATE_SIZE; i++) {
//         for (int j = 0; j < STATE_SIZE; j++) {

//             float kh = 0.0f;

//             for (int k = 0; k < 3; k++) {
//                 kh += K[i][k] * H[k][j];
//             }

//             cov_temp1[i][j] = (i == j ? 1.0f : 0.0f) - kh;
//         }
//     }

   
//     // P_new = (I-KH) * P * (I-KH)^T
    
//     // We calculate one row/column at a time and store the
//     // result back into P only after the required old P values
//     // have been used.
    

//     for (int i = 0; i < STATE_SIZE; i++) {
//         for (int j = 0; j < STATE_SIZE; j++) {
//             cov_temp2[i][j] = P[i][j];
//         }
//     }

//     for (int i = 0; i < STATE_SIZE; i++) {
//         for (int j = 0; j < STATE_SIZE; j++) {

//             float value = 0.0f;

//             for (int k = 0; k < STATE_SIZE; k++) {
//                 for (int l = 0; l < STATE_SIZE; l++) {
//                     value += cov_temp1[i][k] *
//                              cov_temp2[k][l] *
//                              cov_temp1[j][l];
//                 }
//             }

//             // K * R * K^T
//             float krkt = 0.0f;

//             for (int k = 0; k < 3; k++) {
//                 for (int l = 0; l < 3; l++) {
//                     krkt += K[i][k] *
//                             R[k][l] *
//                             K[j][l];
//                 }
//             }

//             P[i][j] = value + krkt;
//         }
//     }
// }





// // Quaternion normalization

// void OurEKF::normalize_quaternion()
// {
//     float qw = x[6];
//     float qx = x[7];
//     float qy = x[8];
//     float qz = x[9];

//     float norm =
//         sqrtf(
//             qw * qw +
//             qx * qx +
//             qy * qy +
//             qz * qz
//         );

//     if (norm > 1.0e-6f) {

//         x[6] /= norm;
//         x[7] /= norm;
//         x[8] /= norm;
//         x[9] /= norm;
//     }
//     else {

//         // Safe fallback
//         x[6] = 1.0f;
//         x[7] = 0.0f;
//         x[8] = 0.0f;
//         x[9] = 0.0f;
//     }
// }



// // Get quaternion


// Quaternion OurEKF::get_state_quaternion() const
// {
//     Quaternion q;

//     q.q1 = x[6];
//     q.q2 = x[7];
//     q.q3 = x[8];
//     q.q4 = x[9];

//     return q;
// }

// // Set quaternion

// void OurEKF::set_state_quaternion(const Quaternion &q)
// {
//     x[6] = q.q1;
//     x[7] = q.q2;
//     x[8] = q.q3;
//     x[9] = q.q4;
// }


// // Position getter

// Vector3f OurEKF::get_position() const
// {
//     return Vector3f(
//         x[0],
//         x[1],
//         x[2]
//     );
// }



// // Velocity getter


// Vector3f OurEKF::get_velocity() const
// {
//     return Vector3f(
//         x[3],
//         x[4],
//         x[5]
//     );
// }



// // Quaternion getter


// Quaternion OurEKF::get_quaternion() const
// {
//     return get_state_quaternion();
// }


// // Accelerometer bias getter


// Vector3f OurEKF::get_accel_bias() const
// {
//     return Vector3f(
//         x[10],
//         x[11],
//         x[12]
//     );
// }


// // ============================================================================
// // Gyro bias getter
// // ============================================================================

// Vector3f OurEKF::get_gyro_bias() const
// {
//     return Vector3f(
//         x[13],
//         x[14],
//         x[15]
//     );
// }


// // ============================================================================
// // Individual getters
// // ============================================================================

// float OurEKF::get_position_x() const
// {
//     return x[0];
// }

// float OurEKF::get_position_y() const
// {
//     return x[1];
// }

// float OurEKF::get_position_z() const
// {
//     return x[2];
// }

// float OurEKF::get_velocity_x() const
// {
//     return x[3];
// }

// float OurEKF::get_velocity_y() const
// {
//     return x[4];
// }

// float OurEKF::get_velocity_z() const
// {
//     return x[5];
// }

// float OurEKF::get_accel_bias_x() const
// {
//     return x[10];
// }

// float OurEKF::get_accel_bias_y() const
// {
//     return x[11];
// }

// float OurEKF::get_accel_bias_z() const
// {
//     return x[12];
// }

// float OurEKF::get_gyro_bias_x() const
// {
//     return x[13];
// }

// float OurEKF::get_gyro_bias_y() const
// {
//     return x[14];
// }

// float OurEKF::get_gyro_bias_z() const
// {
//     return x[15];
// }

// float OurEKF::get_roll() const
// {
//     return roll;
// }

// float OurEKF::get_pitch() const
// {
//     return pitch;
// }

// float OurEKF::get_yaw() const
// {
//     return yaw;
// }


// // ============================================================================
// // Get complete state
// // ============================================================================

// void OurEKF::get_state(float state[STATE_SIZE]) const
// {
//     for (int i = 0; i < STATE_SIZE; i++) {
//         state[i] = x[i];
//     }
// }


// // ============================================================================
// // Get covariance
// // ============================================================================

// void OurEKF::get_covariance(
//     float covariance[STATE_SIZE][STATE_SIZE]) const
// {
//     for (int i = 0; i < STATE_SIZE; i++) {

//         for (int j = 0; j < STATE_SIZE; j++) {

//             covariance[i][j] = P[i][j];
//         }
//     }
// }


// // ============================================================================
// // Innovation getters
// // ============================================================================

// float OurEKF::get_innovation_x() const
// {
//     return innovation[0];
// }

// float OurEKF::get_innovation_y() const
// {
//     return innovation[1];
// }

// float OurEKF::get_innovation_z() const
// {
//     return innovation[2];
// }


// // ============================================================================
// // Is initialized?
// // ============================================================================

// bool OurEKF::is_initialized() const
// {
//     return initialized;
// }


// // ============================================================================
// // Matrix helpers
// // ============================================================================

// void OurEKF::zero_matrix_16(float A[STATE_SIZE][STATE_SIZE])
// {
//     memset(A, 0, sizeof(float) * STATE_SIZE * STATE_SIZE);
// }

// void OurEKF::zero_matrix_16x3(float A[STATE_SIZE][3])
// {
//     memset(A, 0, sizeof(float) * STATE_SIZE * 3);
// }

// void OurEKF::zero_matrix_3x16(float A[3][STATE_SIZE])
// {
//     memset(A, 0, sizeof(float) * 3 * STATE_SIZE);
// }

// void OurEKF::zero_matrix_3(float A[3][3])
// {
//     memset(A, 0, sizeof(float) * 3 * 3);
// }

// void OurEKF::identity_matrix_16(float A[STATE_SIZE][STATE_SIZE])
// {
//     zero_matrix_16(A);

//     for (int i = 0; i < STATE_SIZE; i++) {
//         A[i][i] = 1.0f;
//     }
// }


// // ============================================================================
// // 3x3 matrix inverse
// //
// // A^-1 = adj(A) / det(A)
// //
// // Returns false if determinant is too close to zero.
// // ============================================================================

// bool OurEKF::inverse_3x3(const float A[3][3],
//                          float A_inv[3][3]) const
// {
//     float det =
//           A[0][0] * (A[1][1] * A[2][2] -
//                      A[1][2] * A[2][1])

//         - A[0][1] * (A[1][0] * A[2][2] -
//                      A[1][2] * A[2][0])

//         + A[0][2] * (A[1][0] * A[2][1] -
//                      A[1][1] * A[2][0]);


//     if (fabsf(det) < 1.0e-9f) {
//         return false;
//     }


//     float inv_det = 1.0f / det;


//     A_inv[0][0] =
//         (A[1][1] * A[2][2] -
//          A[1][2] * A[2][1]) * inv_det;

//     A_inv[0][1] =
//         (A[0][2] * A[2][1] -
//          A[0][1] * A[2][2]) * inv_det;

//     A_inv[0][2] =
//         (A[0][1] * A[1][2] -
//          A[0][2] * A[1][1]) * inv_det;


//     A_inv[1][0] =
//         (A[1][2] * A[2][0] -
//          A[1][0] * A[2][2]) * inv_det;

//     A_inv[1][1] =
//         (A[0][0] * A[2][2] -
//          A[0][2] * A[2][0]) * inv_det;

//     A_inv[1][2] =
//         (A[0][2] * A[1][0] -
//          A[0][0] * A[1][2]) * inv_det;


//     A_inv[2][0] =
//         (A[1][0] * A[2][1] -
//          A[1][1] * A[2][0]) * inv_det;

//     A_inv[2][1] =
//         (A[0][1] * A[2][0] -
//          A[0][0] * A[2][1]) * inv_det;

//     A_inv[2][2] =
//         (A[0][0] * A[1][1] -
//          A[0][1] * A[1][0]) * inv_det;

//     return true;
// }


// Vector3f OurEKF::get_accel_body() const
// {
//     return last_accel_body;
// }

// Vector3f OurEKF::get_accel_world() const
// {
//     return last_accel_world;
// }




// #include "ownEkf.h"
// #include <string.h>
// #include <math.h>

// #include <AP_HAL/AP_HAL.h>

//  extern const AP_HAL::HAL& hal;


// // // ============================================================================
// // // Constructor
// // // ============================================================================

// // OurEKF::OurEKF()
// // {
// //     memset(x, 0, sizeof(x));
// //     memset(P, 0, sizeof(P));
// //     memset(F, 0, sizeof(F));
// //     memset(Q, 0, sizeof(Q));

// //     memset(H, 0, sizeof(H));
// //     memset(R, 0, sizeof(R));
// //     memset(S, 0, sizeof(S));
// //     memset(K, 0, sizeof(K));
// //     memset(innovation, 0, sizeof(innovation));

// //     memset(H_mag, 0, sizeof(H_mag));
// //     memset(R_mag, 0, sizeof(R_mag));
// //     memset(S_mag, 0, sizeof(S_mag));
// //     memset(K_mag, 0, sizeof(K_mag));
// //     memset(innovation_mag, 0, sizeof(innovation_mag));

// //     memset(cov_temp1, 0, sizeof(cov_temp1));
// //     memset(cov_temp2, 0, sizeof(cov_temp2));
// //     memset(PHt, 0, sizeof(PHt));

// //     roll  = 0.0f;
// //     pitch = 0.0f;
// //     yaw   = 0.0f;

// //     initialized = false;

// //     calib_active = false;
// //     calib_target = 0;
// //     calib_count = 0;

// //     calib_sum_measured =
// //         Vector3f(0.0f, 0.0f, 0.0f);

// //     calib_sum_expected =
// //         Vector3f(0.0f, 0.0f, 0.0f);

// //     last_accel_body =
// //         Vector3f(0.0f, 0.0f, 0.0f);

// //     last_accel_world =
// //         Vector3f(0.0f, 0.0f, 0.0f);
// // }


// // // ============================================================================
// // // INIT
// // // ============================================================================

// // void OurEKF::init(const Vector3f &position,
// //                   const Vector3f &velocity,
// //                   const Quaternion &quaternion)
// // {
// //     memset(x, 0, sizeof(x));
// //     memset(P, 0, sizeof(P));

// //     // ------------------------------------------------------------------------
// //     // Position
// //     // ------------------------------------------------------------------------

// //     x[0] = position.x;
// //     x[1] = position.y;
// //     x[2] = position.z;

// //     // ------------------------------------------------------------------------
// //     // Velocity
// //     // ------------------------------------------------------------------------

// //     x[3] = velocity.x;
// //     x[4] = velocity.y;
// //     x[5] = velocity.z;

// //     // ------------------------------------------------------------------------
// //     // Quaternion
// //     // ------------------------------------------------------------------------

// //     x[6] = quaternion.q1;
// //     x[7] = quaternion.q2;
// //     x[8] = quaternion.q3;
// //     x[9] = quaternion.q4;

// //     // ------------------------------------------------------------------------
// //     // Accelerometer bias
// //     // ------------------------------------------------------------------------

// //     x[10] = 0.0f;
// //     x[11] = 0.0f;
// //     x[12] = 0.0f;

// //     // ------------------------------------------------------------------------
// //     // Gyroscope bias
// //     // ------------------------------------------------------------------------

// //     x[13] = 0.0f;
// //     x[14] = 0.0f;
// //     x[15] = 0.0f;

// //     // ------------------------------------------------------------------------
// //     // Initial magnetic field.
// //     //
// //     // These values are deliberately simple initial values.
// //     // They are allowed to converge using magnetometer measurements.
// //     // ------------------------------------------------------------------------

// //     x[16] = 0.0f;
// //     x[17] = 0.0f;
// //     x[18] = 0.0f;

// //     x[19] = 0.0f;
// //     x[20] = 0.0f;
// //     x[21] = 0.0f;


// //     have_last_gps = false;

// //     normalize_quaternion();

// //     // ------------------------------------------------------------------------
// //     // Initial covariance
// //     // ------------------------------------------------------------------------

// //     for (int i = 0; i < STATE_SIZE; i++) {
// //         P[i][i] = 0.01f;
// //     }

// //     // Position
// //     P[0][0] = 1.0f;
// //     P[1][1] = 1.0f;
// //     P[2][2] = 1.0f;

// //     // Velocity
// //     P[3][3] = 0.5f;
// //     P[4][4] = 0.5f;
// //     P[5][5] = 0.5f;

// //     // Quaternion
// //     P[6][6] = 0.01f;
// //     P[7][7] = 0.01f;
// //     P[8][8] = 0.01f;
// //     P[9][9] = 0.01f;

// //     // Accelerometer bias
// //     P[10][10] = 0.01f;
// //     P[11][11] = 0.01f;
// //     P[12][12] = 0.01f;

// //     // Gyro bias
// //     P[13][13] = 0.01f;
// //     P[14][14] = 0.01f;
// //     P[15][15] = 0.01f;

// //     // Earth magnetic field uncertainty
// //     P[16][16] = 1.0f;
// //     P[17][17] = 1.0f;
// //     P[18][18] = 1.0f;

// //     // Body magnetic field uncertainty
// //     P[19][19] = 1.0f;
// //     P[20][20] = 1.0f;
// //     P[21][21] = 1.0f;

// //     initialized = true;
// // }


// // // ============================================================================
// // // COMPLETE UPDATE
// // // ============================================================================

// // void OurEKF::update(const Vector3f &accel,
// //                     const Vector3f &gyro,
// //                     const Vector3f &mag,
// //                     float dt,
// //                     bool gps_available,
// //                     const Vector3f &gps_position)
// // {
// //     if (!initialized) {
// //         return;
// //     }

// //     // ------------------------------------------------------------
// //     // Prediction
// //     // ------------------------------------------------------------

// //    predict(accel, gyro, dt);

// //     // ------------------------------------------------------------
// //     // GPS correction
// //     // ------------------------------------------------------------

// //     // if (gps_available) {
// //     //     update_gps(gps_position);
// //     // }

// //     if (gps_available) {

// //     bool new_gps = false;

// //     if (!have_last_gps) {
// //         new_gps = true;
// //     } else {
// //         if ((gps_position - last_gps_position).length() > 0.001f) {
// //             new_gps = true;
// //         }
// //     }

// //     if (new_gps) {

// //         if (update_gps(gps_position)) {
// //             last_gps_position = gps_position;
// //             have_last_gps = true;
// //         }
// //     }
// //   }

// //     // ------------------------------------------------------------
// //     // Magnetometer correction
// //     // ------------------------------------------------------------

// //    update_mag(mag);
// // }


// // // ============================================================================
// // // PREDICTION
// // // ============================================================================

// // void OurEKF::predict(const Vector3f &accel,
// //                      const Vector3f &gyro,
// //                      float dt)
// // {
// //     if (!initialized || dt <= 0.0f) {
// //         return;
// //     }

// //     // ------------------------------------------------------------
// //     // Bias correction
// //     // ------------------------------------------------------------

// //     Vector3f accel_corrected;
// //     Vector3f gyro_corrected;

// //     accel_corrected.x =
// //         accel.x - x[10];

// //     accel_corrected.y =
// //         accel.y - x[11];

// //     accel_corrected.z =
// //         accel.z - x[12];

// //     gyro_corrected.x =
// //         gyro.x - x[13];

// //     gyro_corrected.y =
// //         gyro.y - x[14];

// //     gyro_corrected.z =
// //         gyro.z - x[15];

// //     last_accel_body = accel_corrected;

// //     // ------------------------------------------------------------
// //     // Quaternion prediction
// //     // ------------------------------------------------------------

// //     propagate_quaternion(
// //         gyro_corrected,
// //         dt);

// //     // ------------------------------------------------------------
// //     // Body -> world
// //     // ------------------------------------------------------------

// //     Quaternion q =
// //         get_state_quaternion();

// //     Matrix3f Row =
// //         quaternion_to_rotation_matrix(q);

// //     Vector3f accel_world;

// //     accel_world.x =
// //         Row.a.x * accel_corrected.x +
// //         Row.b.x * accel_corrected.y +
// //         Row.c.x * accel_corrected.z;

// //     accel_world.y =
// //         Row.a.y * accel_corrected.x +
// //         Row.b.y * accel_corrected.y +
// //         Row.c.y * accel_corrected.z;

// //     accel_world.z =
// //         Row.a.z * accel_corrected.x +
// //         Row.b.z * accel_corrected.y +
// //         Row.c.z * accel_corrected.z;

// //     // ------------------------------------------------------------
// //     // Gravity compensation
// //     //
// //     // World +Z = up
// //     // ------------------------------------------------------------

// //     accel_world.z += 9.81f;

// //     last_accel_world = accel_world;

// //     // ------------------------------------------------------------
// //     // Position
// //     // ------------------------------------------------------------

// //     const float half_dt2 =
// //         0.5f * dt * dt;

// //     x[0] +=
// //         x[3] * dt +
// //         accel_world.x * half_dt2;

// //     x[1] +=
// //         x[4] * dt +
// //         accel_world.y * half_dt2;

// //     x[2] +=
// //         x[5] * dt +
// //         accel_world.z * half_dt2;

// //     // ------------------------------------------------------------
// //     // Velocity
// //     // ------------------------------------------------------------

// //     x[3] += accel_world.x * dt;
// //     x[4] += accel_world.y * dt;
// //     x[5] += accel_world.z * dt;

// //     // ------------------------------------------------------------
// //     // Bias means remain unchanged
// //     // ------------------------------------------------------------

// //     // ------------------------------------------------------------
// //     // F
// //     // ------------------------------------------------------------

// //     build_F(
// //         accel_corrected,
// //         dt);

// //     // ------------------------------------------------------------
// //     // Q
// //     // ------------------------------------------------------------

// //     build_Q(dt);

// //     // ------------------------------------------------------------
// //     // Covariance prediction
// //     // ------------------------------------------------------------

// //     predict_covariance();

// //     // ------------------------------------------------------------
// //     // Quaternion normalization
// //     // ------------------------------------------------------------

// //     normalize_quaternion();

// //     Orientation();

// //     /*
// //     hal.console->printf(
// //         "GYRO %.4f %.4f %.4f\n",
// //         gyro.x, gyro.y, gyro.z);

// //     hal.console->printf(
// //         "ACCEL %.4f %.4f %.4f\n",
// //         accel.x, accel.y, accel.z);

// //     hal.console->printf(
// //         "Q %.4f %.4f %.4f %.4f\n",
// //         x[6], x[7], x[8], x[9]);

// //     hal.console->printf(
// //         "RPY %.2f %.2f %.2f\n",
// //         roll * RAD_TO_DEG,
// //         pitch * RAD_TO_DEG,
// //         yaw * RAD_TO_DEG);
// //     */
// // }


// // // ============================================================================
// // // QUATERNION PROPAGATION
// // // ============================================================================

// // void OurEKF::propagate_quaternion(
// //     const Vector3f &gyro,
// //     float dt)
// // {
// //     Quaternion q =
// //         get_state_quaternion();

// //     Quaternion dq =
// //         quaternion_from_gyro(
// //             gyro,
// //             dt);

// //     Quaternion q_new;

// //     q_new.q1 =
// //         q.q1 * dq.q1 -
// //         q.q2 * dq.q2 -
// //         q.q3 * dq.q3 -
// //         q.q4 * dq.q4;

// //     q_new.q2 =
// //         q.q1 * dq.q2 +
// //         q.q2 * dq.q1 +
// //         q.q3 * dq.q4 -
// //         q.q4 * dq.q3;

// //     q_new.q3 =
// //         q.q1 * dq.q3 -
// //         q.q2 * dq.q4 +
// //         q.q3 * dq.q1 +
// //         q.q4 * dq.q2;

// //     q_new.q4 =
// //         q.q1 * dq.q4 +
// //         q.q2 * dq.q3 -
// //         q.q3 * dq.q2 +
// //         q.q4 * dq.q1;

// //     set_state_quaternion(q_new);

// //     normalize_quaternion();
// // }


// // // ============================================================================
// // // DELTA QUATERNION
// // // ============================================================================

// // Quaternion OurEKF::quaternion_from_gyro(
// //     const Vector3f &gyro,
// //     float dt) const
// // {
// //     const float angle_x =
// //         gyro.x * dt;

// //     const float angle_y =
// //         gyro.y * dt;

// //     const float angle_z =
// //         gyro.z * dt;

// //     const float angle =
// //         sqrtf(
// //             angle_x * angle_x +
// //             angle_y * angle_y +
// //             angle_z * angle_z);

// //     Quaternion dq;

// //     if (angle < 1.0e-8f) {

// //         dq.q1 = 1.0f;
// //         dq.q2 = 0.0f;
// //         dq.q3 = 0.0f;
// //         dq.q4 = 0.0f;

// //         return dq;
// //     }

// //     const float half_angle =
// //         0.5f * angle;

// //     const float s =
// //         sinf(half_angle) / angle;

// //     dq.q1 =
// //         cosf(half_angle);

// //     dq.q2 =
// //         angle_x * s;

// //     dq.q3 =
// //         angle_y * s;

// //     dq.q4 =
// //         angle_z * s;

// //     return dq;
// // }


// // // ============================================================================
// // // ORIENTATION
// // // ============================================================================

// // void OurEKF::Orientation()
// // {
// //     Quaternion q =
// //         get_state_quaternion();

// //     const float qw = q.q1;
// //     const float qx = q.q2;
// //     const float qy = q.q3;
// //     const float qz = q.q4;

// //     // Roll

// //     roll =
// //         atan2f(
// //             2.0f *
// //             (qw * qx + qy * qz),
// //             1.0f -
// //             2.0f *
// //             (qx * qx + qy * qy));

// //     // Pitch

// //     float sin_pitch =
// //         2.0f *
// //         (qw * qy - qz * qx);

// //     if (sin_pitch > 1.0f) {
// //         sin_pitch = 1.0f;
// //     }

// //     if (sin_pitch < -1.0f) {
// //         sin_pitch = -1.0f;
// //     }

// //     pitch =
// //         asinf(sin_pitch);

// //     // Yaw

// //     yaw =
// //         atan2f(
// //             2.0f *
// //             (qw * qz + qx * qy),
// //             1.0f -
// //             2.0f *
// //             (qy * qy + qz * qz));
// // }


// // // ============================================================================
// // // QUATERNION -> ROTATION MATRIX
// // // ============================================================================

// // Matrix3f OurEKF::quaternion_to_rotation_matrix(
// //     const Quaternion &q) const
// // {
// //     Matrix3f Row;

// //     const float qw = q.q1;
// //     const float qx = q.q2;
// //     const float qy = q.q3;
// //     const float qz = q.q4;

// //     Row.a.x =
// //         1.0f -
// //         2.0f * (qy * qy + qz * qz);

// //     Row.a.y =
// //         2.0f * (qx * qy + qz * qw);

// //     Row.a.z =
// //         2.0f * (qx * qz - qy * qw);

// //     Row.b.x =
// //         2.0f * (qx * qy - qz * qw);

// //     Row.b.y =
// //         1.0f -
// //         2.0f * (qx * qx + qz * qz);

// //     Row.b.z =
// //         2.0f * (qy * qz + qx * qw);

// //     Row.c.x =
// //         2.0f * (qx * qz + qy * qw);

// //     Row.c.y =
// //         2.0f * (qy * qz - qx * qw);

// //     Row.c.z =
// //         1.0f -
// //         2.0f * (qx * qx + qy * qy);

// //     return Row;
// // }


// // // ============================================================================
// // // F MATRIX
// // // ============================================================================

// // void OurEKF::build_F(
// //     const Vector3f &accel_body,
// //     float dt)
// // {
// //     zero_matrix(F);

// //     // ------------------------------------------------------------
// //     // Identity
// //     // ------------------------------------------------------------

// //     for (int i = 0; i < STATE_SIZE; i++) {
// //         F[i][i] = 1.0f;
// //     }

// //     // ------------------------------------------------------------
// //     // Position <- velocity
// //     // ------------------------------------------------------------

// //     F[0][3] = dt;
// //     F[1][4] = dt;
// //     F[2][5] = dt;

// //     // ------------------------------------------------------------
// //     // Position / velocity <- accel bias
// //     // ------------------------------------------------------------

// //     Quaternion q =
// //         get_state_quaternion();

// //     Matrix3f Row =
// //         quaternion_to_rotation_matrix(q);

// //     const float half_dt2 =
// //         0.5f * dt * dt;

// //     F[0][10] =
// //         -half_dt2 * Row.a.x;

// //     F[0][11] =
// //         -half_dt2 * Row.b.x;

// //     F[0][12] =
// //         -half_dt2 * Row.c.x;

// //     F[1][10] =
// //         -half_dt2 * Row.a.y;

// //     F[1][11] =
// //         -half_dt2 * Row.b.y;

// //     F[1][12] =
// //         -half_dt2 * Row.c.y;

// //     F[2][10] =
// //         -half_dt2 * Row.a.z;

// //     F[2][11] =
// //         -half_dt2 * Row.b.z;

// //     F[2][12] =
// //         -half_dt2 * Row.c.z;

// //     const float dt_r = dt;

// //     F[3][10] =
// //         -dt_r * Row.a.x;

// //     F[3][11] =
// //         -dt_r * Row.b.x;

// //     F[3][12] =
// //         -dt_r * Row.c.x;

// //     F[4][10] =
// //         -dt_r * Row.a.y;

// //     F[4][11] =
// //         -dt_r * Row.b.y;

// //     F[4][12] =
// //         -dt_r * Row.c.y;

// //     F[5][10] =
// //         -dt_r * Row.a.z;

// //     F[5][11] =
// //         -dt_r * Row.b.z;

// //     F[5][12] =
// //         -dt_r * Row.c.z;

// //     // ------------------------------------------------------------
// //     // Quaternion / gyro bias
// //     // ------------------------------------------------------------

// //     const float half_dt =
// //         0.5f * dt;

// //     const float qw = q.q1;
// //     const float qx = q.q2;
// //     const float qy = q.q3;
// //     const float qz = q.q4;

// //     F[6][13] =
// //         half_dt * qx;

// //     F[6][14] =
// //         half_dt * qy;

// //     F[6][15] =
// //         half_dt * qz;

// //     F[7][13] =
// //         -half_dt * qw;

// //     F[7][14] =
// //         half_dt * qz;

// //     F[7][15] =
// //         -half_dt * qy;

// //     F[8][13] =
// //         -half_dt * qz;

// //     F[8][14] =
// //         -half_dt * qw;

// //     F[8][15] =
// //         half_dt * qx;

// //     F[9][13] =
// //         half_dt * qy;

// //     F[9][14] =
// //         -half_dt * qx;

// //     F[9][15] =
// //         -half_dt * qw;

// //     // ------------------------------------------------------------
// //     // Magnetic field states are modeled as slowly varying.
// //     // Their F diagonal remains 1.
// //     // ------------------------------------------------------------

// //     (void)accel_body;
// // }


// // // ============================================================================
// // // Q MATRIX
// // // ============================================================================

// // void OurEKF::build_Q(float dt)
// // {
// //     zero_matrix(Q);

// //     const float accel_noise_std =
// //         0.20f;

// //     const float gyro_noise_std =
// //         0.02f;

// //     const float accel_bias_rw_std =
// //         0.001f;

// //     const float gyro_bias_rw_std =
// //         0.0005f;

// //     // Magnetic field random walk.
// //     //
// //     // These are deliberately small.
// //     // Tune them after the filter is stable.

// //     const float earth_mag_rw_std =
// //         0.001f;

// //     const float body_mag_rw_std =
// //         0.001f;

// //     const float accel_var =
// //         accel_noise_std *
// //         accel_noise_std;

// //     const float gyro_var =
// //         gyro_noise_std *
// //         gyro_noise_std;

// //     const float accel_bias_var =
// //         accel_bias_rw_std *
// //         accel_bias_rw_std;

// //     const float gyro_bias_var =
// //         gyro_bias_rw_std *
// //         gyro_bias_rw_std;

// //     const float earth_mag_var =
// //         earth_mag_rw_std *
// //         earth_mag_rw_std;

// //     const float body_mag_var =
// //         body_mag_rw_std *
// //         body_mag_rw_std;

// //     const float dt2 =
// //         dt * dt;

// //     const float dt4 =
// //         dt2 * dt2;

// //     // Position

// //     const float position_noise =
// //         0.25f *
// //         accel_var *
// //         dt4;

// //     Q[0][0] =
// //         position_noise;

// //     Q[1][1] =
// //         position_noise;

// //     Q[2][2] =
// //         position_noise;

// //     // Velocity

// //     const float velocity_noise =
// //         accel_var * dt2;

// //     Q[3][3] =
// //         velocity_noise;

// //     Q[4][4] =
// //         velocity_noise;

// //     Q[5][5] =
// //         velocity_noise;

// //     // Quaternion

// //     const float quaternion_noise =
// //         0.25f *
// //         gyro_var *
// //         dt2;

// //     Q[6][6] =
// //         quaternion_noise;

// //     Q[7][7] =
// //         quaternion_noise;

// //     Q[8][8] =
// //         quaternion_noise;

// //     Q[9][9] =
// //         quaternion_noise;

// //     // Accelerometer bias

// //     Q[10][10] =
// //         accel_bias_var * dt;

// //     Q[11][11] =
// //         accel_bias_var * dt;

// //     Q[12][12] =
// //         accel_bias_var * dt;

// //     // Gyro bias

// //     Q[13][13] =
// //         gyro_bias_var * dt;

// //     Q[14][14] =
// //         gyro_bias_var * dt;

// //     Q[15][15] =
// //         gyro_bias_var * dt;

// //     // Earth magnetic field

// //     Q[16][16] =
// //         earth_mag_var * dt;

// //     Q[17][17] =
// //         earth_mag_var * dt;

// //     Q[18][18] =
// //         earth_mag_var * dt;

// //     // Body magnetic field

// //     Q[19][19] =
// //         body_mag_var * dt;

// //     Q[20][20] =
// //         body_mag_var * dt;

// //     Q[21][21] =
// //         body_mag_var * dt;
// // }


// // // ============================================================================
// // // COVARIANCE PREDICTION
// // //
// // // P = F P F' + Q
// // //
// // // IMPORTANT:
// // // No 22x22 local arrays are used.
// // // cov_temp1 and cov_temp2 are class members.
// // // ============================================================================

// // void OurEKF::predict_covariance()
// // {
// //     // ------------------------------------------------------------
// //     // temp1 = F * P
// //     // ------------------------------------------------------------

// //     for (int i = 0; i < STATE_SIZE; i++) {

// //         for (int j = 0; j < STATE_SIZE; j++) {

// //             float sum = 0.0f;

// //             for (int k = 0; k < STATE_SIZE; k++) {

// //                 sum +=
// //                     F[i][k] *
// //                     P[k][j];
// //             }

// //             cov_temp1[i][j] =
// //                 sum;
// //         }
// //     }

// //     // ------------------------------------------------------------
// //     // temp2 = temp1 * F'
// //     //              + Q
// //     // ------------------------------------------------------------

// //     for (int i = 0; i < STATE_SIZE; i++) {

// //         for (int j = 0; j < STATE_SIZE; j++) {

// //             float sum = 0.0f;

// //             for (int k = 0; k < STATE_SIZE; k++) {

// //                 sum +=
// //                     cov_temp1[i][k] *
// //                     F[j][k];
// //             }

// //             cov_temp2[i][j] =
// //                 sum + Q[i][j];
// //         }
// //     }

// //     // ------------------------------------------------------------
// //     // Copy back
// //     // ------------------------------------------------------------

// //     for (int i = 0; i < STATE_SIZE; i++) {

// //         for (int j = 0; j < STATE_SIZE; j++) {

// //             P[i][j] =
// //                 cov_temp2[i][j];
// //         }
// //     }
// // }


// // // ============================================================================
// // // GPS H
// // // ============================================================================

// // void OurEKF::build_H()
// // {
// //     zero_matrix_3x(H);

// //     H[0][0] = 1.0f;
// //     H[1][1] = 1.0f;
// //     H[2][2] = 1.0f;
// // }


// // // ============================================================================
// // // GPS R
// // // ============================================================================

// // void OurEKF::build_R()
// // {
// //     zero_matrix_3(R);

// //     const float gps_x_std =
// //         2.0f;

// //     const float gps_y_std =
// //         2.0f;

// //     const float gps_z_std =
// //         4.0f;

// //     R[0][0] =
// //         gps_x_std *
// //         gps_x_std;

// //     R[1][1] =
// //         gps_y_std *
// //         gps_y_std;

// //     R[2][2] =
// //         gps_z_std *
// //         gps_z_std;
// // }


// // // ============================================================================
// // // GPS UPDATE
// // // ============================================================================

// // bool OurEKF::update_gps(
// //     const Vector3f &gps_position)
// // {
// //     if (!initialized) {
// //         return false;
// //     }

// //     build_H();

// //     // ------------------------------------------------------------
// //     // Innovation
// //     // ------------------------------------------------------------

// //     innovation[0] =
// //         gps_position.x - x[0];

// //     innovation[1] =
// //         gps_position.y - x[1];

// //     innovation[2] =
// //         gps_position.z - x[2];

// //     build_R();

// //     if (!calculate_innovation_covariance()) {
// //         return false;
// //     }

// //     if (!calculate_kalman_gain()) {
// //         return false;
// //     }

// //     correct_state();

// //     correct_covariance();

// //     normalize_quaternion();

// //     return true;
// // }



// // // ============================================================================
// // // GPS S
// // // ============================================================================

// // bool OurEKF::calculate_innovation_covariance()
// // {
// //     zero_matrix_3(S);

// //     for (int i = 0; i < GPS_SIZE; i++) {

// //         for (int j = 0; j < GPS_SIZE; j++) {

// //             float value = 0.0f;

// //             for (int k = 0; k < STATE_SIZE; k++) {

// //                 for (int l = 0; l < STATE_SIZE; l++) {

// //                     value +=
// //                         H[i][k] *
// //                         P[k][l] *
// //                         H[j][l];
// //                 }
// //             }

// //             S[i][j] =
// //                 value + R[i][j];
// //         }
// //     }

// //     return true;
// // }


// // // ============================================================================
// // // GPS K
// // // ============================================================================

// // bool OurEKF::calculate_kalman_gain()
// // {
// //     float S_inv[3][3];

// //     memset(
// //         S_inv,
// //         0,
// //         sizeof(S_inv));

// //     if (!inverse_3x3(
// //             S,
// //             S_inv)) {

// //         return false;
// //     }

// //     // ------------------------------------------------------------
// //     // PH'
// //     //
// //     // Stored in class member to avoid stack growth.
// //     // ------------------------------------------------------------

// //     memset(
// //         PHt,
// //         0,
// //         sizeof(PHt));

// //     for (int i = 0; i < STATE_SIZE; i++) {

// //         for (int j = 0; j < GPS_SIZE; j++) {

// //             float value = 0.0f;

// //             for (int k = 0; k < STATE_SIZE; k++) {

// //                 value +=
// //                     P[i][k] *
// //                     H[j][k];
// //             }

// //             PHt[i][j] =
// //                 value;
// //         }
// //     }

// //     // ------------------------------------------------------------
// //     // K = PH' S^-1
// //     // ------------------------------------------------------------

// //     for (int i = 0; i < STATE_SIZE; i++) {

// //         for (int j = 0; j < GPS_SIZE; j++) {

// //             float value = 0.0f;

// //             for (int k = 0; k < GPS_SIZE; k++) {

// //                 value +=
// //                     PHt[i][k] *
// //                     S_inv[k][j];
// //             }

// //             K[i][j] =
// //                 value;
// //         }
// //     }

// //     return true;
// // }


// // // ============================================================================
// // // GPS STATE CORRECTION
// // // ============================================================================

// // void OurEKF::correct_state()
// // {
// //     // Calculate correction directly.
// //     //
// //     // No large local array.

// //     for (int i = 0; i < STATE_SIZE; i++) {

// //         float dx = 0.0f;

// //         for (int j = 0; j < GPS_SIZE; j++) {

// //             dx +=
// //                 K[i][j] *
// //                 innovation[j];
// //         }

// //         x[i] += dx;
// //     }
// // }


// // // ============================================================================
// // // GPS COVARIANCE CORRECTION
// // //
// // // Joseph form:
// // //
// // // P = (I-KH) P (I-KH)' + K R K'
// // //
// // // cov_temp1 and cov_temp2 are persistent class members.
// // // ============================================================================

// // void OurEKF::correct_covariance()
// // {
// //     // ------------------------------------------------------------
// //     // cov_temp1 = I - KH
// //     // ------------------------------------------------------------

// //     for (int i = 0; i < STATE_SIZE; i++) {

// //         for (int j = 0; j < STATE_SIZE; j++) {

// //             float kh = 0.0f;

// //             for (int k = 0; k < GPS_SIZE; k++) {

// //                 kh +=
// //                     K[i][k] *
// //                     H[k][j];
// //             }

// //             cov_temp1[i][j] =
// //                 (i == j ? 1.0f : 0.0f)
// //                 - kh;
// //         }
// //     }

// //     // ------------------------------------------------------------
// //     // Preserve old P
// //     // ------------------------------------------------------------

// //     for (int i = 0; i < STATE_SIZE; i++) {

// //         for (int j = 0; j < STATE_SIZE; j++) {

// //             cov_temp2[i][j] =
// //                 P[i][j];
// //         }
// //     }

// //     // ------------------------------------------------------------
// //     // Calculate Joseph form
// //     // ------------------------------------------------------------

// //     for (int i = 0; i < STATE_SIZE; i++) {

// //         for (int j = 0; j < STATE_SIZE; j++) {

// //             float value = 0.0f;

// //             for (int k = 0; k < STATE_SIZE; k++) {

// //                 for (int l = 0; l < STATE_SIZE; l++) {

// //                     value +=
// //                         cov_temp1[i][k] *
// //                         cov_temp2[k][l] *
// //                         cov_temp1[j][l];
// //                 }
// //             }

// //             float krkt = 0.0f;

// //             for (int k = 0; k < GPS_SIZE; k++) {

// //                 for (int l = 0; l < GPS_SIZE; l++) {

// //                     krkt +=
// //                         K[i][k] *
// //                         R[k][l] *
// //                         K[j][l];
// //                 }
// //             }

// //             P[i][j] =
// //                 value + krkt;
// //         }
// //     }
// // }


// // // ============================================================================
// // // MAGNETOMETER PREDICTION
// // //
// // // State:
// // //
// // // Earth magnetic field:
// // //   x[16], x[17], x[18]
// // //
// // // Body magnetic field:
// // //   x[19], x[20], x[21]
// // //
// // // Measurement model:
// // //
// // // predicted_body_mag = R' * earth_mag + body_mag
// // //
// // // R is body -> world.
// // // Therefore R' converts world -> body.
// // // ============================================================================

// // Vector3f OurEKF::predict_magnetometer(
// //     const Quaternion &q) const
// // {
// //     Matrix3f Row =
// //         quaternion_to_rotation_matrix(q);

// //     const float ex = x[16];
// //     const float ey = x[17];
// //     const float ez = x[18];

// //     const float bx = x[19];
// //     const float by = x[20];
// //     const float bz = x[21];

// //     Vector3f predicted;

// //     // R transpose * Earth field

// //     predicted.x =
// //         Row.a.x * ex +
// //         Row.a.y * ey +
// //         Row.a.z * ez +
// //         bx;

// //     predicted.y =
// //         Row.b.x * ex +
// //         Row.b.y * ey +
// //         Row.b.z * ez +
// //         by;

// //     predicted.z =
// //         Row.c.x * ex +
// //         Row.c.y * ey +
// //         Row.c.z * ez +
// //         bz;

// //     return predicted;
// // }


// // // ============================================================================
// // // MAGNETOMETER JACOBIAN
// // //
// // // Numerical Jacobian:
// // //
// // // H = d(predicted_mag) / d(state)
// // //
// // // This is intentionally used first because it lets us verify the complete
// // // magnetic measurement model without introducing a large generated analytical
// // // Jacobian.
// // //
// // // Only these states influence the magnetic prediction:
// // //
// // // quaternion 6..9
// // // earth field 16..18
// // // body field 19..21
// // // ============================================================================

// // void OurEKF::build_mag_measurement_jacobian(
// //     const Vector3f &mag_prediction)
// // {
// //     (void)mag_prediction;

// //     for (int row = 0; row < MAG_SIZE; row++) {

// //         for (int col = 0; col < STATE_SIZE; col++) {

// //             H_mag[row][col] =
// //                 0.0f;
// //         }
// //     }

// //     Quaternion q =
// //         get_state_quaternion();

// //     /*
// //      * Small perturbation used for numerical derivative.
// //      *
// //      * This is a state-space derivative, not a sensor-noise parameter.
// //      */

// //     const float eps =
// //         1.0e-5f;

// //     // ------------------------------------------------------------
// //     // Quaternion derivatives
// //     // ------------------------------------------------------------

// //     for (int q_index = 6;
// //          q_index <= 9;
// //          q_index++) {

// //         const float original =
// //             x[q_index];

// //         x[q_index] =
// //             original + eps;

// //         Quaternion qp =
// //             get_state_quaternion();

// //         Vector3f mp =
// //             predict_magnetometer(qp);

// //         x[q_index] =
// //             original - eps;

// //         Quaternion qm =
// //             get_state_quaternion();

// //         Vector3f mm =
// //             predict_magnetometer(qm);

// //         x[q_index] =
// //             original;

// //         H_mag[0][q_index] =
// //             (mp.x - mm.x) /
// //             (2.0f * eps);

// //         H_mag[1][q_index] =
// //             (mp.y - mm.y) /
// //             (2.0f * eps);

// //         H_mag[2][q_index] =
// //             (mp.z - mm.z) /
// //             (2.0f * eps);
// //     }

// //     // ------------------------------------------------------------
// //     // Earth magnetic field derivatives
// //     //
// //     // R' * Earth
// //     // ------------------------------------------------------------

// //     Matrix3f Row =
// //         quaternion_to_rotation_matrix(q);

// //     H_mag[0][16] = Row.a.x;
// //     H_mag[0][17] = Row.a.y;
// //     H_mag[0][18] = Row.a.z;

// //     H_mag[1][16] = Row.b.x;
// //     H_mag[1][17] = Row.b.y;
// //     H_mag[1][18] = Row.b.z;

// //     H_mag[2][16] = Row.c.x;
// //     H_mag[2][17] = Row.c.y;
// //     H_mag[2][18] = Row.c.z;

// //     // ------------------------------------------------------------
// //     // Body magnetic field is directly added.
// //     // ------------------------------------------------------------

// //     H_mag[0][19] = 1.0f;
// //     H_mag[1][20] = 1.0f;
// //     H_mag[2][21] = 1.0f;
// // }


// // // ============================================================================
// // // MAGNETOMETER UPDATE
// // //
// // // Three-axis measurement:
// // //
// // // z = [mx my mz]
// // //
// // // We fuse each component sequentially.
// // // ============================================================================

// // bool OurEKF::update_mag(
// //     const Vector3f &mag)
// // {
// //     if (!initialized) {
// //         return false;
// //     }

// //     Quaternion q =
// //         get_state_quaternion();

// //     Vector3f predicted =
// //         predict_magnetometer(q);

// //     // ------------------------------------------------------------
// //     // Build Jacobian
// //     // ------------------------------------------------------------

// //     build_mag_measurement_jacobian(
// //         predicted);

// //     // ------------------------------------------------------------
// //     // Measurement covariance
// //     //
// //     // This is an example value.
// //     // It must eventually be tuned to the actual magnetometer.
// //     // ------------------------------------------------------------

// //     zero_matrix_3(R_mag);

// //     const float mag_noise_std =
// //         0.05f;

// //     const float mag_var =
// //         mag_noise_std *
// //         mag_noise_std;

// //     R_mag[0][0] = mag_var;
// //     R_mag[1][1] = mag_var;
// //     R_mag[2][2] = mag_var;

// //     // ------------------------------------------------------------
// //     // Innovation
// //     // ------------------------------------------------------------

// //     innovation_mag[0] =
// //         mag.x - predicted.x;

// //     innovation_mag[1] =
// //         mag.y - predicted.y;

// //     innovation_mag[2] =
// //         mag.z - predicted.z;

// //     // ------------------------------------------------------------
// //     // Sequentially fuse X
// //     // ------------------------------------------------------------

// //     if (!fuse_mag_component(
// //             mag,
// //             0)) {

// //         return false;
// //     }

// //     // ------------------------------------------------------------
// //     // Sequentially fuse Y
// //     // ------------------------------------------------------------

// //     if (!fuse_mag_component(
// //             mag,
// //             1)) {

// //         return false;
// //     }

// //     // ------------------------------------------------------------
// //     // Sequentially fuse Z
// //     // ------------------------------------------------------------

// //     if (!fuse_mag_component(
// //             mag,
// //             2)) {

// //         return false;
// //     }

// //     normalize_quaternion();

// //     Orientation();

// //     return true;
// // }


// // // ============================================================================
// // // SINGLE MAGNETOMETER COMPONENT FUSION
// // //
// // // S = H P H' + R
// // //
// // // K = P H' / S
// // //
// // // x = x + K innovation
// // //
// // // P = (I-KH)P
// // //
// // // Only one measurement row is fused at a time.
// // // ============================================================================

// // bool OurEKF::fuse_mag_component(
// //     const Vector3f &mag,
// //     uint8_t component)
// // {
// //     if (component >= 3) {
// //         return false;
// //     }

// //     // ------------------------------------------------------------
// //     // Recalculate prediction because the previous component may
// //     // have changed the state.
// //     // ------------------------------------------------------------

// //     Quaternion q =
// //         get_state_quaternion();

// //     Vector3f predicted =
// //         predict_magnetometer(q);

// //     // ------------------------------------------------------------
// //     // Rebuild numerical Jacobian after previous correction.
// //     // ------------------------------------------------------------

// //     build_mag_measurement_jacobian(
// //         predicted);

// //     float z = 0.0f;
// //     float zhat = 0.0f;

// //     if (component == 0) {
// //         z = mag.x;
// //         zhat = predicted.x;
// //     }
// //     else if (component == 1) {
// //         z = mag.y;
// //         zhat = predicted.y;
// //     }
// //     else {
// //         z = mag.z;
// //         zhat = predicted.z;
// //     }

// //     const float innovation1 =
// //         z - zhat;

// //     innovation_mag[component] =
// //         innovation1;

// //     // ------------------------------------------------------------
// //     // Measurement Jacobian row
// //     // ------------------------------------------------------------

// //     // S = HPH' + R
// //     // Since this is a scalar measurement:

// //     float S_value =
// //         R_mag[component][component];

// //     for (int i = 0; i < STATE_SIZE; i++) {

// //         for (int j = 0; j < STATE_SIZE; j++) {

// //             S_value +=
// //                 H_mag[component][i] *
// //                 P[i][j] *
// //                 H_mag[component][j];
// //         }
// //     }

// //     if (S_value <= 1.0e-12f) {
// //         return false;
// //     }

// //     S_mag[component][component] =
// //         S_value;

// //     // ------------------------------------------------------------
// //     // K = PH' / S
// //     // ------------------------------------------------------------

// //     for (int i = 0; i < STATE_SIZE; i++) {

// //         float value = 0.0f;

// //         for (int j = 0; j < STATE_SIZE; j++) {

// //             value +=
// //                 P[i][j] *
// //                 H_mag[component][j];
// //         }

// //         K_mag[i][component] =
// //             value / S_value;
// //     }

// //     // ------------------------------------------------------------
// //     // State correction
// //     // ------------------------------------------------------------

// //     for (int i = 0; i < STATE_SIZE; i++) {

// //         x[i] +=
// //             K_mag[i][component] *
// //             innovation1;
// //     }

// //     // ------------------------------------------------------------
// //     // Quaternion normalization
// //     // ------------------------------------------------------------

// //     normalize_quaternion();

// //     // ------------------------------------------------------------
// //     // Covariance:
// //     //
// //     // Pnew = (I-KH)P
// //     //
// //     // We use cov_temp1 as Pnew.
// //     // No local 22x22 array.
// //     // ------------------------------------------------------------

// //     for (int i = 0; i < STATE_SIZE; i++) {

// //         for (int j = 0; j < STATE_SIZE; j++) {

// //             float value =
// //                 P[i][j];

// //             float khp =
// //                 0.0f;

// //             for (int k = 0; k < STATE_SIZE; k++) {

// //                 khp +=
// //                     K_mag[i][component] *
// //                     H_mag[component][k] *
// //                     P[k][j];
// //             }

// //             value -= khp;

// //             cov_temp1[i][j] =
// //                 value;
// //         }
// //     }

// //     // ------------------------------------------------------------
// //     // Copy covariance
// //     // ------------------------------------------------------------

// //     for (int i = 0; i < STATE_SIZE; i++) {

// //         for (int j = 0; j < STATE_SIZE; j++) {

// //             P[i][j] =
// //                 cov_temp1[i][j];
// //         }
// //     }

// //     return true;
// // }


// // // ============================================================================
// // // QUATERNION NORMALIZATION
// // // ============================================================================

// // void OurEKF::normalize_quaternion()
// // {
// //     float qw = x[6];
// //     float qx = x[7];
// //     float qy = x[8];
// //     float qz = x[9];

// //     float norm =
// //         sqrtf(
// //             qw * qw +
// //             qx * qx +
// //             qy * qy +
// //             qz * qz);

// //     if (norm > 1.0e-6f) {

// //         x[6] /= norm;
// //         x[7] /= norm;
// //         x[8] /= norm;
// //         x[9] /= norm;
// //     }
// //     else {

// //         x[6] = 1.0f;
// //         x[7] = 0.0f;
// //         x[8] = 0.0f;
// //         x[9] = 0.0f;
// //     }
// // }


// // // ============================================================================
// // // GET / SET QUATERNION
// // // ============================================================================

// // Quaternion OurEKF::get_state_quaternion() const
// // {
// //     Quaternion q;

// //     q.q1 = x[6];
// //     q.q2 = x[7];
// //     q.q3 = x[8];
// //     q.q4 = x[9];

// //     return q;
// // }


// // void OurEKF::set_state_quaternion(
// //     const Quaternion &q)
// // {
// //     x[6] = q.q1;
// //     x[7] = q.q2;
// //     x[8] = q.q3;
// //     x[9] = q.q4;
// // }


// // // ============================================================================
// // // GETTERS
// // // ============================================================================

// // Vector3f OurEKF::get_position() const
// // {
// //     return Vector3f(
// //         x[0],
// //         x[1],
// //         x[2]);
// // }


// // Vector3f OurEKF::get_velocity() const
// // {
// //     return Vector3f(
// //         x[3],
// //         x[4],
// //         x[5]);
// // }


// // Quaternion OurEKF::get_quaternion() const
// // {
// //     return get_state_quaternion();
// // }


// // Vector3f OurEKF::get_accel_bias() const
// // {
// //     return Vector3f(
// //         x[10],
// //         x[11],
// //         x[12]);
// // }


// // Vector3f OurEKF::get_gyro_bias() const
// // {
// //     return Vector3f(
// //         x[13],
// //         x[14],
// //         x[15]);
// // }


// // Vector3f OurEKF::get_accel_body() const
// // {
// //     return last_accel_body;
// // }


// // Vector3f OurEKF::get_accel_world() const
// // {
// //     return last_accel_world;
// // }


// // // ============================================================================
// // // INDIVIDUAL GETTERS
// // // ============================================================================

// // float OurEKF::get_position_x() const
// // {
// //     return x[0];
// // }

// // float OurEKF::get_position_y() const
// // {
// //     return x[1];
// // }

// // float OurEKF::get_position_z() const
// // {
// //     return x[2];
// // }

// // float OurEKF::get_velocity_x() const
// // {
// //     return x[3];
// // }

// // float OurEKF::get_velocity_y() const
// // {
// //     return x[4];
// // }

// // float OurEKF::get_velocity_z() const
// // {
// //     return x[5];
// // }

// // float OurEKF::get_accel_bias_x() const
// // {
// //     return x[10];
// // }

// // float OurEKF::get_accel_bias_y() const
// // {
// //     return x[11];
// // }

// // float OurEKF::get_accel_bias_z() const
// // {
// //     return x[12];
// // }

// // float OurEKF::get_gyro_bias_x() const
// // {
// //     return x[13];
// // }

// // float OurEKF::get_gyro_bias_y() const
// // {
// //     return x[14];
// // }

// // float OurEKF::get_gyro_bias_z() const
// // {
// //     return x[15];
// // }

// // float OurEKF::get_roll() const
// // {
// //     return roll;
// // }

// // float OurEKF::get_pitch() const
// // {
// //     return pitch;
// // }

// // float OurEKF::get_yaw() const
// // {
// //     return yaw;
// // }


// // // ============================================================================
// // // COMPLETE STATE
// // // ============================================================================

// // void OurEKF::get_state(
// //     float state[STATE_SIZE]) const
// // {
// //     for (int i = 0; i < STATE_SIZE; i++) {
// //         state[i] = x[i];
// //     }
// // }


// // // ============================================================================
// // // COMPLETE COVARIANCE
// // // ============================================================================

// // void OurEKF::get_covariance(
// //     float covariance[STATE_SIZE][STATE_SIZE]) const
// // {
// //     for (int i = 0; i < STATE_SIZE; i++) {

// //         for (int j = 0; j < STATE_SIZE; j++) {

// //             covariance[i][j] =
// //                 P[i][j];
// //         }
// //     }
// // }


// // // ============================================================================
// // // INNOVATION GETTERS
// // // ============================================================================

// // float OurEKF::get_innovation_x() const
// // {
// //     return innovation[0];
// // }

// // float OurEKF::get_innovation_y() const
// // {
// //     return innovation[1];
// // }

// // float OurEKF::get_innovation_z() const
// // {
// //     return innovation[2];
// // }


// // // ============================================================================
// // // INITIALIZED
// // // ============================================================================

// // bool OurEKF::is_initialized() const
// // {
// //     return initialized;
// // }


// // // ============================================================================
// // // MATRIX HELPERS
// // // ============================================================================

// // void OurEKF::zero_matrix(
// //     float A[STATE_SIZE][STATE_SIZE])
// // {
// //     memset(
// //         A,
// //         0,
// //         sizeof(float) *
// //         STATE_SIZE *
// //         STATE_SIZE);
// // }


// // void OurEKF::zero_matrix_x3(
// //     float A[STATE_SIZE][3])
// // {
// //     memset(
// //         A,
// //         0,
// //         sizeof(float) *
// //         STATE_SIZE *
// //         3);
// // }


// // void OurEKF::zero_matrix_3x(
// //     float A[3][STATE_SIZE])
// // {
// //     memset(
// //         A,
// //         0,
// //         sizeof(float) *
// //         3 *
// //         STATE_SIZE);
// // }


// // void OurEKF::zero_matrix_3(
// //     float A[3][3])
// // {
// //     memset(
// //         A,
// //         0,
// //         sizeof(float) *
// //         3 *
// //         3);
// // }


// // void OurEKF::identity_matrix(
// //     float A[STATE_SIZE][STATE_SIZE])
// // {
// //     zero_matrix(A);

// //     for (int i = 0; i < STATE_SIZE; i++) {
// //         A[i][i] = 1.0f;
// //     }
// // }


// // // ============================================================================
// // // 3x3 INVERSE
// // // ============================================================================

// // bool OurEKF::inverse_3x3(
// //     const float A[3][3],
// //     float A_inv[3][3]) const
// // {
// //     const float det =
// //           A[0][0] *
// //           (A[1][1] * A[2][2] -
// //            A[1][2] * A[2][1])

// //         - A[0][1] *
// //           (A[1][0] * A[2][2] -
// //            A[1][2] * A[2][0])

// //         + A[0][2] *
// //           (A[1][0] * A[2][1] -
// //            A[1][1] * A[2][0]);

// //     if (fabsf(det) < 1.0e-9f) {
// //         return false;
// //     }

// //     const float inv_det =
// //         1.0f / det;

// //     A_inv[0][0] =
// //         (A[1][1] * A[2][2] -
// //          A[1][2] * A[2][1]) *
// //         inv_det;

// //     A_inv[0][1] =
// //         (A[0][2] * A[2][1] -
// //          A[0][1] * A[2][2]) *
// //         inv_det;

// //     A_inv[0][2] =
// //         (A[0][1] * A[1][2] -
// //          A[0][2] * A[1][1]) *
// //         inv_det;

// //     A_inv[1][0] =
// //         (A[1][2] * A[2][0] -
// //          A[1][0] * A[2][2]) *
// //         inv_det;

// //     A_inv[1][1] =
// //         (A[0][0] * A[2][2] -
// //          A[0][2] * A[2][0]) *
// //         inv_det;

// //     A_inv[1][2] =
// //         (A[0][2] * A[1][0] -
// //          A[0][0] * A[1][2]) *
// //         inv_det;

// //     A_inv[2][0] =
// //         (A[1][0] * A[2][1] -
// //          A[1][1] * A[2][0]) *
// //         inv_det;

// //     A_inv[2][1] =
// //         (A[0][1] * A[2][0] -
// //          A[0][0] * A[2][1]) *
// //         inv_det;

// //     A_inv[2][2] =
// //         (A[0][0] * A[1][1] -
// //          A[0][1] * A[1][0]) *
// //         inv_det;

// //     return true;
// // }


// #include "ownEkf.h"

// #include <string.h>
// #include <math.h>

// #include <AP_HAL/AP_HAL.h>

// extern const AP_HAL::HAL& hal;


// // ============================================================================
// // Constructor
// // ============================================================================

// OurEKF::OurEKF()
// {
//     memset(x, 0, sizeof(x));
//     memset(P, 0, sizeof(P));
//     memset(F, 0, sizeof(F));
//     memset(Q, 0, sizeof(Q));

//     memset(H, 0, sizeof(H));
//     memset(R, 0, sizeof(R));
//     memset(S, 0, sizeof(S));
//     memset(K, 0, sizeof(K));
//     memset(innovation, 0, sizeof(innovation));

//     memset(H_mag, 0, sizeof(H_mag));
//     memset(R_mag, 0, sizeof(R_mag));
//     memset(S_mag, 0, sizeof(S_mag));
//     memset(K_mag, 0, sizeof(K_mag));
//     memset(innovation_mag, 0, sizeof(innovation_mag));

//     memset(cov_temp1, 0, sizeof(cov_temp1));
//     memset(cov_temp2, 0, sizeof(cov_temp2));
//     memset(PHt, 0, sizeof(PHt));

//     roll  = 0.0f;
//     pitch = 0.0f;
//     yaw   = 0.0f;

//     initialized = false;

//     calib_active = false;
//     calib_target = 0;
//     calib_count = 0;

//     calib_sum_measured =
//         Vector3f(0.0f, 0.0f, 0.0f);

//     calib_sum_expected =
//         Vector3f(0.0f, 0.0f, 0.0f);

//     last_accel_body =
//         Vector3f(0.0f, 0.0f, 0.0f);

//     last_accel_world =
//         Vector3f(0.0f, 0.0f, 0.0f);
// }


// // ============================================================================
// // INIT
// // ============================================================================

// void OurEKF::init(const Vector3f &position,
//                   const Vector3f &velocity,
//                   const Quaternion &quaternion)
// {
//     memset(x, 0, sizeof(x));
//     memset(P, 0, sizeof(P));

//     // ------------------------------------------------------------------------
//     // Position
//     // ------------------------------------------------------------------------

//     x[0] = position.x;
//     x[1] = position.y;
//     x[2] = position.z;

//     // ------------------------------------------------------------------------
//     // Velocity
//     // ------------------------------------------------------------------------

//     x[3] = velocity.x;
//     x[4] = velocity.y;
//     x[5] = velocity.z;

//     // ------------------------------------------------------------------------
//     // Quaternion
//     // ------------------------------------------------------------------------

//     x[6] = quaternion.q1;
//     x[7] = quaternion.q2;
//     x[8] = quaternion.q3;
//     x[9] = quaternion.q4;

//     // ------------------------------------------------------------------------
//     // Accelerometer bias
//     // ------------------------------------------------------------------------

//     x[10] = 0.0f;
//     x[11] = 0.0f;
//     x[12] = 0.0f;

//     // ------------------------------------------------------------------------
//     // Gyroscope bias
//     // ------------------------------------------------------------------------

//     x[13] = 0.0f;
//     x[14] = 0.0f;
//     x[15] = 0.0f;

//     // ------------------------------------------------------------------------
//     // Initial magnetic field.
//     //
//     // These values are deliberately simple initial values.
//     // They are allowed to converge using magnetometer measurements.
//     // ------------------------------------------------------------------------

//     x[16] = 0.0f;
//     x[17] = 0.0f;
//     x[18] = 0.0f;

//     x[19] = 0.0f;
//     x[20] = 0.0f;
//     x[21] = 0.0f;


//     have_last_gps = false;

//     normalize_quaternion();

//     // ------------------------------------------------------------------------
//     // Initial covariance
//     // ------------------------------------------------------------------------

//     for (int i = 0; i < STATE_SIZE; i++) {
//         P[i][i] = 0.01f;
//     }

//     // Position
//     P[0][0] = 1.0f;
//     P[1][1] = 1.0f;
//     P[2][2] = 1.0f;

//     // Velocity
//     P[3][3] = 0.5f;
//     P[4][4] = 0.5f;
//     P[5][5] = 0.5f;

//     // Quaternion
//     P[6][6] = 0.01f;
//     P[7][7] = 0.01f;
//     P[8][8] = 0.01f;
//     P[9][9] = 0.01f;

//     // Accelerometer bias
//     P[10][10] = 0.01f;
//     P[11][11] = 0.01f;
//     P[12][12] = 0.01f;

//     // Gyro bias
//     P[13][13] = 0.01f;
//     P[14][14] = 0.01f;
//     P[15][15] = 0.01f;

//     // Earth magnetic field uncertainty
//     P[16][16] = 1.0f;
//     P[17][17] = 1.0f;
//     P[18][18] = 1.0f;

//     // Body magnetic field uncertainty
//     P[19][19] = 1.0f;
//     P[20][20] = 1.0f;
//     P[21][21] = 1.0f;

//     initialized = true;
// }


// // ============================================================================
// // COMPLETE UPDATE
// // ============================================================================

// void OurEKF::update(const Vector3f &accel,
//                     const Vector3f &gyro,
//                     const Vector3f &mag,
//                     float dt,
//                     bool gps_available,
//                     const Vector3f &gps_position)
// {
//     if (!initialized) {
//         return;
//     }

//     // ------------------------------------------------------------
//     // Prediction
//     // ------------------------------------------------------------

//    predict(accel, gyro, dt);

//     // ------------------------------------------------------------
//     // GPS correction
//     // ------------------------------------------------------------

//     // if (gps_available) {
//     //     update_gps(gps_position);
//     // }

//     if (gps_available) {

//     bool new_gps = false;

//     if (!have_last_gps) {
//         new_gps = true;
//     } else {
//         if ((gps_position - last_gps_position).length() > 0.001f) {
//             new_gps = true;
//         }
//     }

//     if (new_gps) {

//         if (update_gps(gps_position)) {
//             last_gps_position = gps_position;
//             have_last_gps = true;
//         }
//     }
//   }

//     // ------------------------------------------------------------
//     // Magnetometer correction
//     // ------------------------------------------------------------

//    update_mag(mag);
// }


// // ============================================================================
// // PREDICTION
// // ============================================================================

// void OurEKF::predict(const Vector3f &accel,
//                      const Vector3f &gyro,
//                      float dt)
// {
//     if (!initialized) {
//         return;
//     }

//     if (!(dt > 0.0f) || dt > 0.1f) {
//         return;
//     }

//     // IMU bias correction
//     Vector3f accel_corrected;
//     accel_corrected.x = accel.x - x[10];
//     accel_corrected.y = accel.y - x[11];
//     accel_corrected.z = accel.z - x[12];

//     Vector3f gyro_corrected;
//     gyro_corrected.x = gyro.x - x[13];
//     gyro_corrected.y = gyro.y - x[14];
//     gyro_corrected.z = gyro.z - x[15];

//     last_accel_body = accel_corrected;

//     // Quaternion prediction
//     propagate_quaternion(gyro_corrected, dt);

//     // Body -> local NED
//     const Quaternion q = get_state_quaternion();
//     const Matrix3f Rbn = quaternion_to_rotation_matrix(q);

//     Vector3f accel_ned;
//     accel_ned.x = Rbn.a.x * accel_corrected.x +
//                   Rbn.a.y * accel_corrected.y +
//                   Rbn.a.z * accel_corrected.z;
//     accel_ned.y = Rbn.b.x * accel_corrected.x +
//                   Rbn.b.y * accel_corrected.y +
//                   Rbn.b.z * accel_corrected.z;
//     accel_ned.z = Rbn.c.x * accel_corrected.x +
//                   Rbn.c.y * accel_corrected.y +
//                   Rbn.c.z * accel_corrected.z;

//     // ArduPilot uses NED: gravity is +Z. IMU measures specific force.
//     accel_ned.z += 9.80665f;

//     last_accel_world = accel_ned;

//     // Position and velocity prediction
//     const float half_dt2 = 0.5f * dt * dt;
//     x[0] += x[3] * dt + accel_ned.x * half_dt2;
//     x[1] += x[4] * dt + accel_ned.y * half_dt2;
//     x[2] += x[5] * dt + accel_ned.z * half_dt2;

//     x[3] += accel_ned.x * dt;
//     x[4] += accel_ned.y * dt;
//     x[5] += accel_ned.z * dt;

//     // Covariance prediction
//     build_F(accel_corrected, dt);
//     build_Q(dt);
//     predict_covariance();
//     normalize_quaternion();

//     /*
//      * Gravity fusion for tilt stability.
//      * This is only enabled when the accelerometer magnitude is close to 1g,
//      * so vehicle manoeuvres are not treated as a gravity measurement.
//      * We sequentially fuse X/Y/Z, reusing the scalar mag work buffers.
//      */
//     const float acc_norm = accel_corrected.length();
//     if (acc_norm > 8.0f && acc_norm < 11.5f) {
//         for (uint8_t axis = 0; axis < 3; axis++) {
//             const Quaternion qg = get_state_quaternion();
//             const Matrix3f Rg = quaternion_to_rotation_matrix(qg);

//             const float z = (axis == 0) ? accel_corrected.x :
//                             (axis == 1) ? accel_corrected.y :
//                                           accel_corrected.z;

//             const float zhat = (axis == 0) ? -9.80665f * Rg.a.z :
//                                 (axis == 1) ? -9.80665f * Rg.b.z :
//                                               -9.80665f * Rg.c.z;

//             const float innov_g = z - zhat;
//             const float eps_q = 1.0e-5f;

//             for (int i = 0; i < STATE_SIZE; i++) {
//                 H_mag[0][i] = 0.0f;
//             }

//             // Numerical derivative of gravity observation wrt quaternion.
//             for (int qi = 6; qi <= 9; qi++) {
//                 const float old = x[qi];

//                 x[qi] = old + eps_q;
//                 normalize_quaternion();
//                 const Matrix3f Rp = quaternion_to_rotation_matrix(get_state_quaternion());
//                 const float hp = (axis == 0) ? -9.80665f * Rp.a.z :
//                                  (axis == 1) ? -9.80665f * Rp.b.z :
//                                                -9.80665f * Rp.c.z;

//                 x[qi] = old - eps_q;
//                 normalize_quaternion();
//                 const Matrix3f Rm = quaternion_to_rotation_matrix(get_state_quaternion());
//                 const float hm = (axis == 0) ? -9.80665f * Rm.a.z :
//                                  (axis == 1) ? -9.80665f * Rm.b.z :
//                                                -9.80665f * Rm.c.z;

//                 x[qi] = old;
//                 normalize_quaternion();

//                 H_mag[0][qi] = (hp - hm) / (2.0f * eps_q);
//             }

//             const float accel_obs_std = 0.35f;
//             const float Rg_var = accel_obs_std * accel_obs_std;

//             float Sg = Rg_var;
//             for (int i = 0; i < STATE_SIZE; i++) {
//                 for (int j = 0; j < STATE_SIZE; j++) {
//                     Sg += H_mag[0][i] * P[i][j] * H_mag[0][j];
//                 }
//             }

//             if (Sg > 1.0e-9f && isfinite(Sg) &&
//                 innov_g * innov_g <= 16.0f * Sg) {

//                 for (int i = 0; i < STATE_SIZE; i++) {
//                     float ph = 0.0f;
//                     for (int j = 0; j < STATE_SIZE; j++) {
//                         ph += P[i][j] * H_mag[0][j];
//                     }
//                     K_mag[i][0] = ph / Sg;
//                 }

//                 for (int i = 0; i < STATE_SIZE; i++) {
//                     x[i] += K_mag[i][0] * innov_g;
//                 }
//                 normalize_quaternion();

//                 for (int i = 0; i < STATE_SIZE; i++) {
//                     for (int j = 0; j < STATE_SIZE; j++) {
//                         float v = P[i][j];
//                         for (int k = 0; k < STATE_SIZE; k++) {
//                             v -= K_mag[i][0] * H_mag[0][k] * P[k][j];
//                         }
//                         cov_temp1[i][j] = v;
//                     }
//                 }
//                 for (int i = 0; i < STATE_SIZE; i++) {
//                     for (int j = 0; j < STATE_SIZE; j++) {
//                         P[i][j] = cov_temp1[i][j];
//                     }
//                 }
//             }
//         }
//     }

//     Orientation();
// }


// // ============================================================================
// // QUATERNION PROPAGATION
// // ============================================================================

// void OurEKF::propagate_quaternion(
//     const Vector3f &gyro,
//     float dt)
// {
//     Quaternion q =
//         get_state_quaternion();

//     Quaternion dq =
//         quaternion_from_gyro(
//             gyro,
//             dt);

//     Quaternion q_new;

//     q_new.q1 =
//         q.q1 * dq.q1 -
//         q.q2 * dq.q2 -
//         q.q3 * dq.q3 -
//         q.q4 * dq.q4;

//     q_new.q2 =
//         q.q1 * dq.q2 +
//         q.q2 * dq.q1 +
//         q.q3 * dq.q4 -
//         q.q4 * dq.q3;

//     q_new.q3 =
//         q.q1 * dq.q3 -
//         q.q2 * dq.q4 +
//         q.q3 * dq.q1 +
//         q.q4 * dq.q2;

//     q_new.q4 =
//         q.q1 * dq.q4 +
//         q.q2 * dq.q3 -
//         q.q3 * dq.q2 +
//         q.q4 * dq.q1;

//     set_state_quaternion(q_new);

//     normalize_quaternion();
// }


// // ============================================================================
// // DELTA QUATERNION
// // ============================================================================

// Quaternion OurEKF::quaternion_from_gyro(
//     const Vector3f &gyro,
//     float dt) const
// {
//     const float angle_x =
//         gyro.x * dt;

//     const float angle_y =
//         gyro.y * dt;

//     const float angle_z =
//         gyro.z * dt;

//     const float angle =
//         sqrtf(
//             angle_x * angle_x +
//             angle_y * angle_y +
//             angle_z * angle_z);

//     Quaternion dq;

//     if (angle < 1.0e-8f) {

//         dq.q1 = 1.0f;
//         dq.q2 = 0.0f;
//         dq.q3 = 0.0f;
//         dq.q4 = 0.0f;

//         return dq;
//     }

//     const float half_angle =
//         0.5f * angle;

//     const float s =
//         sinf(half_angle) / angle;

//     dq.q1 =
//         cosf(half_angle);

//     dq.q2 =
//         angle_x * s;

//     dq.q3 =
//         angle_y * s;

//     dq.q4 =
//         angle_z * s;

//     return dq;
// }


// // ============================================================================
// // ORIENTATION
// // ============================================================================

// void OurEKF::Orientation()
// {
//     Quaternion q =
//         get_state_quaternion();

//     const float qw = q.q1;
//     const float qx = q.q2;
//     const float qy = q.q3;
//     const float qz = q.q4;

//     // Roll

//     roll =
//         atan2f(
//             2.0f *
//             (qw * qx + qy * qz),
//             1.0f -
//             2.0f *
//             (qx * qx + qy * qy));

//     // Pitch

//     float sin_pitch =
//         2.0f *
//         (qw * qy - qz * qx);

//     if (sin_pitch > 1.0f) {
//         sin_pitch = 1.0f;
//     }

//     if (sin_pitch < -1.0f) {
//         sin_pitch = -1.0f;
//     }

//     pitch =
//         asinf(sin_pitch);

//     // Yaw

//     yaw =
//         atan2f(
//             2.0f *
//             (qw * qz + qx * qy),
//             1.0f -
//             2.0f *
//             (qy * qy + qz * qz));
// }


// // ============================================================================
// // QUATERNION -> ROTATION MATRIX
// // ============================================================================

// Matrix3f OurEKF::quaternion_to_rotation_matrix(
//     const Quaternion &q) const
// {
//     Matrix3f Row;

//     const float qw = q.q1;
//     const float qx = q.q2;
//     const float qy = q.q3;
//     const float qz = q.q4;

//     Row.a.x =
//         1.0f -
//         2.0f * (qy * qy + qz * qz);

//     Row.a.y =
//         2.0f * (qx * qy + qz * qw);

//     Row.a.z =
//         2.0f * (qx * qz - qy * qw);

//     Row.b.x =
//         2.0f * (qx * qy - qz * qw);

//     Row.b.y =
//         1.0f -
//         2.0f * (qx * qx + qz * qz);

//     Row.b.z =
//         2.0f * (qy * qz + qx * qw);

//     Row.c.x =
//         2.0f * (qx * qz + qy * qw);

//     Row.c.y =
//         2.0f * (qy * qz - qx * qw);

//     Row.c.z =
//         1.0f -
//         2.0f * (qx * qx + qy * qy);

//     return Row;
// }


// // ============================================================================
// // F MATRIX
// // ============================================================================

// void OurEKF::build_F(
//     const Vector3f &accel_body,
//     float dt)
// {
//     zero_matrix(F);

//     // ------------------------------------------------------------
//     // Identity
//     // ------------------------------------------------------------

//     for (int i = 0; i < STATE_SIZE; i++) {
//         F[i][i] = 1.0f;
//     }

//     // ------------------------------------------------------------
//     // Position <- velocity
//     // ------------------------------------------------------------

//     F[0][3] = dt;
//     F[1][4] = dt;
//     F[2][5] = dt;

//     // ------------------------------------------------------------
//     // Position / velocity <- accel bias
//     // ------------------------------------------------------------

//     Quaternion q =
//         get_state_quaternion();

//     Matrix3f Row =
//         quaternion_to_rotation_matrix(q);

//     const float half_dt2 =
//         0.5f * dt * dt;

//     F[0][10] =
//         -half_dt2 * Row.a.x;

//     F[0][11] =
//         -half_dt2 * Row.b.x;

//     F[0][12] =
//         -half_dt2 * Row.c.x;

//     F[1][10] =
//         -half_dt2 * Row.a.y;

//     F[1][11] =
//         -half_dt2 * Row.b.y;

//     F[1][12] =
//         -half_dt2 * Row.c.y;

//     F[2][10] =
//         -half_dt2 * Row.a.z;

//     F[2][11] =
//         -half_dt2 * Row.b.z;

//     F[2][12] =
//         -half_dt2 * Row.c.z;

//     const float dt_r = dt;

//     F[3][10] =
//         -dt_r * Row.a.x;

//     F[3][11] =
//         -dt_r * Row.b.x;

//     F[3][12] =
//         -dt_r * Row.c.x;

//     F[4][10] =
//         -dt_r * Row.a.y;

//     F[4][11] =
//         -dt_r * Row.b.y;

//     F[4][12] =
//         -dt_r * Row.c.y;

//     F[5][10] =
//         -dt_r * Row.a.z;

//     F[5][11] =
//         -dt_r * Row.b.z;

//     F[5][12] =
//         -dt_r * Row.c.z;

//     // ------------------------------------------------------------
//     // Quaternion / gyro bias
//     // ------------------------------------------------------------

//     const float half_dt =
//         0.5f * dt;

//     const float qw = q.q1;
//     const float qx = q.q2;
//     const float qy = q.q3;
//     const float qz = q.q4;

//     F[6][13] =
//         half_dt * qx;

//     F[6][14] =
//         half_dt * qy;

//     F[6][15] =
//         half_dt * qz;

//     F[7][13] =
//         -half_dt * qw;

//     F[7][14] =
//         half_dt * qz;

//     F[7][15] =
//         -half_dt * qy;

//     F[8][13] =
//         -half_dt * qz;

//     F[8][14] =
//         -half_dt * qw;

//     F[8][15] =
//         half_dt * qx;

//     F[9][13] =
//         half_dt * qy;

//     F[9][14] =
//         -half_dt * qx;

//     F[9][15] =
//         -half_dt * qw;

//     // ------------------------------------------------------------
//     // Magnetic field states are modeled as slowly varying.
//     // Their F diagonal remains 1.
//     // ------------------------------------------------------------

//     (void)accel_body;
// }


// // ============================================================================
// // Q MATRIX
// // ============================================================================

// void OurEKF::build_Q(float dt)
// {
//     zero_matrix(Q);

//     const float accel_noise_std =
//         0.20f;

//     const float gyro_noise_std =
//         0.02f;

//     const float accel_bias_rw_std =
//         0.001f;

//     const float gyro_bias_rw_std =
//         0.0005f;

//     // Magnetic field random walk.
//     //
//     // These are deliberately small.
//     // Tune them after the filter is stable.

//     const float earth_mag_rw_std =
//         0.001f;

//     const float body_mag_rw_std =
//         0.001f;

//     const float accel_var =
//         accel_noise_std *
//         accel_noise_std;

//     const float gyro_var =
//         gyro_noise_std *
//         gyro_noise_std;

//     const float accel_bias_var =
//         accel_bias_rw_std *
//         accel_bias_rw_std;

//     const float gyro_bias_var =
//         gyro_bias_rw_std *
//         gyro_bias_rw_std;

//     const float earth_mag_var =
//         earth_mag_rw_std *
//         earth_mag_rw_std;

//     const float body_mag_var =
//         body_mag_rw_std *
//         body_mag_rw_std;

//     const float dt2 =
//         dt * dt;

//     const float dt4 =
//         dt2 * dt2;

//     // Position

//     const float position_noise =
//         0.25f *
//         accel_var *
//         dt4;

//     Q[0][0] =
//         position_noise;

//     Q[1][1] =
//         position_noise;

//     Q[2][2] =
//         position_noise;

//     // Velocity

//     const float velocity_noise =
//         accel_var * dt2;

//     Q[3][3] =
//         velocity_noise;

//     Q[4][4] =
//         velocity_noise;

//     Q[5][5] =
//         velocity_noise;

//     // Quaternion

//     const float quaternion_noise =
//         0.25f *
//         gyro_var *
//         dt2;

//     Q[6][6] =
//         quaternion_noise;

//     Q[7][7] =
//         quaternion_noise;

//     Q[8][8] =
//         quaternion_noise;

//     Q[9][9] =
//         quaternion_noise;

//     // Accelerometer bias

//     Q[10][10] =
//         accel_bias_var * dt;

//     Q[11][11] =
//         accel_bias_var * dt;

//     Q[12][12] =
//         accel_bias_var * dt;

//     // Gyro bias

//     Q[13][13] =
//         gyro_bias_var * dt;

//     Q[14][14] =
//         gyro_bias_var * dt;

//     Q[15][15] =
//         gyro_bias_var * dt;

//     // Earth magnetic field

//     Q[16][16] =
//         earth_mag_var * dt;

//     Q[17][17] =
//         earth_mag_var * dt;

//     Q[18][18] =
//         earth_mag_var * dt;

//     // Body magnetic field

//     Q[19][19] =
//         body_mag_var * dt;

//     Q[20][20] =
//         body_mag_var * dt;

//     Q[21][21] =
//         body_mag_var * dt;
// }


// // ============================================================================
// // COVARIANCE PREDICTION
// //
// // P = F P F' + Q
// //
// // IMPORTANT:
// // No 22x22 local arrays are used.
// // cov_temp1 and cov_temp2 are class members.
// // ============================================================================

// void OurEKF::predict_covariance()
// {
//     // ------------------------------------------------------------
//     // temp1 = F * P
//     // ------------------------------------------------------------

//     for (int i = 0; i < STATE_SIZE; i++) {

//         for (int j = 0; j < STATE_SIZE; j++) {

//             float sum = 0.0f;

//             for (int k = 0; k < STATE_SIZE; k++) {

//                 sum +=
//                     F[i][k] *
//                     P[k][j];
//             }

//             cov_temp1[i][j] =
//                 sum;
//         }
//     }

//     // ------------------------------------------------------------
//     // temp2 = temp1 * F'
//     //              + Q
//     // ------------------------------------------------------------

//     for (int i = 0; i < STATE_SIZE; i++) {

//         for (int j = 0; j < STATE_SIZE; j++) {

//             float sum = 0.0f;

//             for (int k = 0; k < STATE_SIZE; k++) {

//                 sum +=
//                     cov_temp1[i][k] *
//                     F[j][k];
//             }

//             cov_temp2[i][j] =
//                 sum + Q[i][j];
//         }
//     }

//     // ------------------------------------------------------------
//     // Copy back
//     // ------------------------------------------------------------

//     for (int i = 0; i < STATE_SIZE; i++) {

//         for (int j = 0; j < STATE_SIZE; j++) {

//             P[i][j] =
//                 cov_temp2[i][j];
//         }
//     }
// }


// // ============================================================================
// // GPS H
// // ============================================================================

// void OurEKF::build_H()
// {
//     zero_matrix_3x(H);

//     H[0][0] = 1.0f;
//     H[1][1] = 1.0f;
//     H[2][2] = 1.0f;
// }


// // ============================================================================
// // GPS R
// // ============================================================================

// void OurEKF::build_R()
// {
//     zero_matrix_3(R);

//     const float gps_x_std =
//         2.0f;

//     const float gps_y_std =
//         2.0f;

//     const float gps_z_std =
//         4.0f;

//     R[0][0] =
//         gps_x_std *
//         gps_x_std;

//     R[1][1] =
//         gps_y_std *
//         gps_y_std;

//     R[2][2] =
//         gps_z_std *
//         gps_z_std;
// }


// // ============================================================================
// // GPS UPDATE
// // ============================================================================

// bool OurEKF::update_gps(const Vector3f &gps_position)
// {
//     if (!initialized) {
//         return false;
//     }

//     build_H();
//     build_R();

//     // ArduPilot-style sequential scalar fusion: N, E, D position axes.
//     for (uint8_t axis = 0; axis < 3; axis++) {
//         const float z = (axis == 0) ? gps_position.x :
//                         (axis == 1) ? gps_position.y :
//                                       gps_position.z;
//         const float innov = z - x[axis];
//         innovation[axis] = innov;

//         // H for this scalar observation is 1 at the corresponding position state.
//         const float S_axis = P[axis][axis] + R[axis][axis];
//         if (!(S_axis > 1.0e-9f) || !isfinite(S_axis)) {
//             return false;
//         }

//         S[axis][axis] = S_axis;

//         for (int i = 0; i < STATE_SIZE; i++) {
//             K[i][0] = P[i][axis] / S_axis;
//         }

//         // Innovation gate. 5-sigma is conservative for this custom filter.
//         if (innov * innov > 25.0f * S_axis) {
//             continue;
//         }

//         // State correction.
//         for (int i = 0; i < STATE_SIZE; i++) {
//             x[i] += K[i][0] * innov;
//         }
//         normalize_quaternion();

//         // P = (I-KH)P, with H selecting one state.
//         for (int i = 0; i < STATE_SIZE; i++) {
//             for (int j = 0; j < STATE_SIZE; j++) {
//                 cov_temp1[i][j] = P[i][j] - K[i][0] * P[axis][j];
//             }
//         }
//         for (int i = 0; i < STATE_SIZE; i++) {
//             for (int j = 0; j < STATE_SIZE; j++) {
//                 P[i][j] = cov_temp1[i][j];
//             }
//         }

//         // Keep covariance symmetric and non-negative on the diagonal.
//         for (int i = 0; i < STATE_SIZE; i++) {
//             if (P[i][i] < 1.0e-12f) {
//                 P[i][i] = 1.0e-12f;
//             }
//             for (int j = i + 1; j < STATE_SIZE; j++) {
//                 const float v = 0.5f * (P[i][j] + P[j][i]);
//                 P[i][j] = v;
//                 P[j][i] = v;
//             }
//         }
//     }

//     last_gps_position = gps_position;
//     have_last_gps = true;
//     return true;
// }


// // ============================================================================
// // GPS S
// // ============================================================================

// bool OurEKF::calculate_innovation_covariance()
// {
//     zero_matrix_3(S);

//     for (int i = 0; i < GPS_SIZE; i++) {

//         for (int j = 0; j < GPS_SIZE; j++) {

//             float value = 0.0f;

//             for (int k = 0; k < STATE_SIZE; k++) {

//                 for (int l = 0; l < STATE_SIZE; l++) {

//                     value +=
//                         H[i][k] *
//                         P[k][l] *
//                         H[j][l];
//                 }
//             }

//             S[i][j] =
//                 value + R[i][j];
//         }
//     }

//     return true;
// }


// // ============================================================================
// // GPS K
// // ============================================================================

// bool OurEKF::calculate_kalman_gain()
// {
//     float S_inv[3][3];

//     memset(
//         S_inv,
//         0,
//         sizeof(S_inv));

//     if (!inverse_3x3(
//             S,
//             S_inv)) {

//         return false;
//     }

//     // ------------------------------------------------------------
//     // PH'
//     //
//     // Stored in class member to avoid stack growth.
//     // ------------------------------------------------------------

//     memset(
//         PHt,
//         0,
//         sizeof(PHt));

//     for (int i = 0; i < STATE_SIZE; i++) {

//         for (int j = 0; j < GPS_SIZE; j++) {

//             float value = 0.0f;

//             for (int k = 0; k < STATE_SIZE; k++) {

//                 value +=
//                     P[i][k] *
//                     H[j][k];
//             }

//             PHt[i][j] =
//                 value;
//         }
//     }

//     // ------------------------------------------------------------
//     // K = PH' S^-1
//     // ------------------------------------------------------------

//     for (int i = 0; i < STATE_SIZE; i++) {

//         for (int j = 0; j < GPS_SIZE; j++) {

//             float value = 0.0f;

//             for (int k = 0; k < GPS_SIZE; k++) {

//                 value +=
//                     PHt[i][k] *
//                     S_inv[k][j];
//             }

//             K[i][j] =
//                 value;
//         }
//     }

//     return true;
// }


// // ============================================================================
// // GPS STATE CORRECTION
// // ============================================================================

// void OurEKF::correct_state()
// {
//     // Calculate correction directly.
//     //
//     // No large local array.

//     for (int i = 0; i < STATE_SIZE; i++) {

//         float dx = 0.0f;

//         for (int j = 0; j < GPS_SIZE; j++) {

//             dx +=
//                 K[i][j] *
//                 innovation[j];
//         }

//         x[i] += dx;
//     }
// }


// // ============================================================================
// // GPS COVARIANCE CORRECTION
// //
// // Joseph form:
// //
// // P = (I-KH) P (I-KH)' + K R K'
// //
// // cov_temp1 and cov_temp2 are persistent class members.
// // ============================================================================

// void OurEKF::correct_covariance()
// {
//     // ------------------------------------------------------------
//     // cov_temp1 = I - KH
//     // ------------------------------------------------------------

//     for (int i = 0; i < STATE_SIZE; i++) {

//         for (int j = 0; j < STATE_SIZE; j++) {

//             float kh = 0.0f;

//             for (int k = 0; k < GPS_SIZE; k++) {

//                 kh +=
//                     K[i][k] *
//                     H[k][j];
//             }

//             cov_temp1[i][j] =
//                 (i == j ? 1.0f : 0.0f)
//                 - kh;
//         }
//     }

//     // ------------------------------------------------------------
//     // Preserve old P
//     // ------------------------------------------------------------

//     for (int i = 0; i < STATE_SIZE; i++) {

//         for (int j = 0; j < STATE_SIZE; j++) {

//             cov_temp2[i][j] =
//                 P[i][j];
//         }
//     }

//     // ------------------------------------------------------------
//     // Calculate Joseph form
//     // ------------------------------------------------------------

//     for (int i = 0; i < STATE_SIZE; i++) {

//         for (int j = 0; j < STATE_SIZE; j++) {

//             float value = 0.0f;

//             for (int k = 0; k < STATE_SIZE; k++) {

//                 for (int l = 0; l < STATE_SIZE; l++) {

//                     value +=
//                         cov_temp1[i][k] *
//                         cov_temp2[k][l] *
//                         cov_temp1[j][l];
//                 }
//             }

//             float krkt = 0.0f;

//             for (int k = 0; k < GPS_SIZE; k++) {

//                 for (int l = 0; l < GPS_SIZE; l++) {

//                     krkt +=
//                         K[i][k] *
//                         R[k][l] *
//                         K[j][l];
//                 }
//             }

//             P[i][j] =
//                 value + krkt;
//         }
//     }
// }


// // ============================================================================
// // MAGNETOMETER PREDICTION
// //
// // State:
// //
// // Earth magnetic field:
// //   x[16], x[17], x[18]
// //
// // Body magnetic field:
// //   x[19], x[20], x[21]
// //
// // Measurement model:
// //
// // predicted_body_mag = R' * earth_mag + body_mag
// //
// // R is body -> world.
// // Therefore R' converts world -> body.
// // ============================================================================

// Vector3f OurEKF::predict_magnetometer(
//     const Quaternion &q) const
// {
//     Matrix3f Row =
//         quaternion_to_rotation_matrix(q);

//     const float ex = x[16];
//     const float ey = x[17];
//     const float ez = x[18];

//     const float bx = x[19];
//     const float by = x[20];
//     const float bz = x[21];

//     Vector3f predicted;

//     // R transpose * Earth field

//     predicted.x =
//         Row.a.x * ex +
//         Row.a.y * ey +
//         Row.a.z * ez +
//         bx;

//     predicted.y =
//         Row.b.x * ex +
//         Row.b.y * ey +
//         Row.b.z * ez +
//         by;

//     predicted.z =
//         Row.c.x * ex +
//         Row.c.y * ey +
//         Row.c.z * ez +
//         bz;

//     return predicted;
// }


// // ============================================================================
// // MAGNETOMETER JACOBIAN
// //
// // Numerical Jacobian:
// //
// // H = d(predicted_mag) / d(state)
// //
// // This is intentionally used first because it lets us verify the complete
// // magnetic measurement model without introducing a large generated analytical
// // Jacobian.
// //
// // Only these states influence the magnetic prediction:
// //
// // quaternion 6..9
// // earth field 16..18
// // body field 19..21
// // ============================================================================

// void OurEKF::build_mag_measurement_jacobian(
//     const Vector3f &mag_prediction)
// {
//     (void)mag_prediction;

//     for (int row = 0; row < MAG_SIZE; row++) {

//         for (int col = 0; col < STATE_SIZE; col++) {

//             H_mag[row][col] =
//                 0.0f;
//         }
//     }

//     Quaternion q =
//         get_state_quaternion();

//     /*
//      * Small perturbation used for numerical derivative.
//      *
//      * This is a state-space derivative, not a sensor-noise parameter.
//      */

//     const float eps =
//         1.0e-5f;

//     // ------------------------------------------------------------
//     // Quaternion derivatives
//     // ------------------------------------------------------------

//     for (int q_index = 6;
//          q_index <= 9;
//          q_index++) {

//         const float original =
//             x[q_index];

//         x[q_index] =
//             original + eps;

//         Quaternion qp =
//             get_state_quaternion();

//         Vector3f mp =
//             predict_magnetometer(qp);

//         x[q_index] =
//             original - eps;

//         Quaternion qm =
//             get_state_quaternion();

//         Vector3f mm =
//             predict_magnetometer(qm);

//         x[q_index] =
//             original;

//         H_mag[0][q_index] =
//             (mp.x - mm.x) /
//             (2.0f * eps);

//         H_mag[1][q_index] =
//             (mp.y - mm.y) /
//             (2.0f * eps);

//         H_mag[2][q_index] =
//             (mp.z - mm.z) /
//             (2.0f * eps);
//     }

//     // ------------------------------------------------------------
//     // Earth magnetic field derivatives
//     //
//     // R' * Earth
//     // ------------------------------------------------------------

//     Matrix3f Row =
//         quaternion_to_rotation_matrix(q);

//     H_mag[0][16] = Row.a.x;
//     H_mag[0][17] = Row.a.y;
//     H_mag[0][18] = Row.a.z;

//     H_mag[1][16] = Row.b.x;
//     H_mag[1][17] = Row.b.y;
//     H_mag[1][18] = Row.b.z;

//     H_mag[2][16] = Row.c.x;
//     H_mag[2][17] = Row.c.y;
//     H_mag[2][18] = Row.c.z;

//     // ------------------------------------------------------------
//     // Body magnetic field is directly added.
//     // ------------------------------------------------------------

//     H_mag[0][19] = 1.0f;
//     H_mag[1][20] = 1.0f;
//     H_mag[2][21] = 1.0f;
// }


// // ============================================================================
// // MAGNETOMETER UPDATE
// //
// // Three-axis measurement:
// //
// // z = [mx my mz]
// //
// // We fuse each component sequentially.
// // ============================================================================

// bool OurEKF::update_mag(
//     const Vector3f &mag)
// {
//     if (!initialized) {
//         return false;
//     }

//     const float mag_norm =
//         sqrtf(mag.x * mag.x + mag.y * mag.y + mag.z * mag.z);

//     if (!(mag_norm > 1.0e-4f) || !isfinite(mag_norm)) {
//         return false;
//     }

//     // Initialise the earth-field state from the first valid compass sample.
//     // Without this, the filter starts with a zero predicted magnetic field.
//     const float earth_norm =
//         sqrtf(x[16] * x[16] + x[17] * x[17] + x[18] * x[18]);

//     if (earth_norm < 1.0e-4f) {
//         const Matrix3f R0 = quaternion_to_rotation_matrix(get_state_quaternion());
//         x[16] = R0.a.x * mag.x + R0.b.x * mag.y + R0.c.x * mag.z;
//         x[17] = R0.a.y * mag.x + R0.b.y * mag.y + R0.c.y * mag.z;
//         x[18] = R0.a.z * mag.x + R0.b.z * mag.y + R0.c.z * mag.z;
//     }

//     Quaternion q =
//         get_state_quaternion();

//     Vector3f predicted =
//         predict_magnetometer(q);

//     // ------------------------------------------------------------
//     // Build Jacobian
//     // ------------------------------------------------------------

//     build_mag_measurement_jacobian(
//         predicted);

//     // ------------------------------------------------------------
//     // Measurement covariance
//     //
//     // This is an example value.
//     // It must eventually be tuned to the actual magnetometer.
//     // ------------------------------------------------------------

//     zero_matrix_3(R_mag);

//     const float mag_noise_std =
//         0.05f;

//     const float mag_var =
//         mag_noise_std *
//         mag_noise_std;

//     R_mag[0][0] = mag_var;
//     R_mag[1][1] = mag_var;
//     R_mag[2][2] = mag_var;

//     // ------------------------------------------------------------
//     // Innovation
//     // ------------------------------------------------------------

//     innovation_mag[0] =
//         mag.x - predicted.x;

//     innovation_mag[1] =
//         mag.y - predicted.y;

//     innovation_mag[2] =
//         mag.z - predicted.z;

//     // ------------------------------------------------------------
//     // Sequentially fuse X
//     // ------------------------------------------------------------

//     if (!fuse_mag_component(
//             mag,
//             0)) {

//         return false;
//     }

//     // ------------------------------------------------------------
//     // Sequentially fuse Y
//     // ------------------------------------------------------------

//     if (!fuse_mag_component(
//             mag,
//             1)) {

//         return false;
//     }

//     // ------------------------------------------------------------
//     // Sequentially fuse Z
//     // ------------------------------------------------------------

//     if (!fuse_mag_component(
//             mag,
//             2)) {

//         return false;
//     }

//     normalize_quaternion();

//     Orientation();

//     return true;
// }


// // ============================================================================
// // SINGLE MAGNETOMETER COMPONENT FUSION
// //
// // S = H P H' + R
// //
// // K = P H' / S
// //
// // x = x + K innovation
// //
// // P = (I-KH)P
// //
// // Only one measurement row is fused at a time.
// // ============================================================================

// bool OurEKF::fuse_mag_component(
//     const Vector3f &mag,
//     uint8_t component)
// {
//     if (component >= 3) {
//         return false;
//     }

//     // ------------------------------------------------------------
//     // Recalculate prediction because the previous component may
//     // have changed the state.
//     // ------------------------------------------------------------

//     Quaternion q =
//         get_state_quaternion();

//     Vector3f predicted =
//         predict_magnetometer(q);

//     // ------------------------------------------------------------
//     // Rebuild numerical Jacobian after previous correction.
//     // ------------------------------------------------------------

//     build_mag_measurement_jacobian(
//         predicted);

//     float z = 0.0f;
//     float zhat = 0.0f;

//     if (component == 0) {
//         z = mag.x;
//         zhat = predicted.x;
//     }
//     else if (component == 1) {
//         z = mag.y;
//         zhat = predicted.y;
//     }
//     else {
//         z = mag.z;
//         zhat = predicted.z;
//     }

//     const float innovation1 =
//         z - zhat;

//     innovation_mag[component] =
//         innovation1;

//     // ------------------------------------------------------------
//     // Measurement Jacobian row
//     // ------------------------------------------------------------

//     // S = HPH' + R
//     // Since this is a scalar measurement:

//     float S_value =
//         R_mag[component][component];

//     for (int i = 0; i < STATE_SIZE; i++) {

//         for (int j = 0; j < STATE_SIZE; j++) {

//             S_value +=
//                 H_mag[component][i] *
//                 P[i][j] *
//                 H_mag[component][j];
//         }
//     }

//     if (S_value <= 1.0e-12f) {
//         return false;
//     }

//     S_mag[component][component] =
//         S_value;

//     // ------------------------------------------------------------
//     // K = PH' / S
//     // ------------------------------------------------------------

//     for (int i = 0; i < STATE_SIZE; i++) {

//         float value = 0.0f;

//         for (int j = 0; j < STATE_SIZE; j++) {

//             value +=
//                 P[i][j] *
//                 H_mag[component][j];
//         }

//         K_mag[i][component] =
//             value / S_value;
//     }

//     // ------------------------------------------------------------
//     // State correction
//     // ------------------------------------------------------------

//     for (int i = 0; i < STATE_SIZE; i++) {

//         x[i] +=
//             K_mag[i][component] *
//             innovation1;
//     }

//     // ------------------------------------------------------------
//     // Quaternion normalization
//     // ------------------------------------------------------------

//     normalize_quaternion();

//     // ------------------------------------------------------------
//     // Covariance:
//     //
//     // Pnew = (I-KH)P
//     //
//     // We use cov_temp1 as Pnew.
//     // No local 22x22 array.
//     // ------------------------------------------------------------

//     for (int i = 0; i < STATE_SIZE; i++) {

//         for (int j = 0; j < STATE_SIZE; j++) {

//             float value =
//                 P[i][j];

//             float khp =
//                 0.0f;

//             for (int k = 0; k < STATE_SIZE; k++) {

//                 khp +=
//                     K_mag[i][component] *
//                     H_mag[component][k] *
//                     P[k][j];
//             }

//             value -= khp;

//             cov_temp1[i][j] =
//                 value;
//         }
//     }

//     // ------------------------------------------------------------
//     // Copy covariance
//     // ------------------------------------------------------------

//     for (int i = 0; i < STATE_SIZE; i++) {

//         for (int j = 0; j < STATE_SIZE; j++) {

//             P[i][j] =
//                 cov_temp1[i][j];
//         }
//     }

//     return true;
// }


// // ============================================================================
// // QUATERNION NORMALIZATION
// // ============================================================================

// void OurEKF::normalize_quaternion()
// {
//     float qw = x[6];
//     float qx = x[7];
//     float qy = x[8];
//     float qz = x[9];

//     float norm =
//         sqrtf(
//             qw * qw +
//             qx * qx +
//             qy * qy +
//             qz * qz);

//     if (norm > 1.0e-6f) {

//         x[6] /= norm;
//         x[7] /= norm;
//         x[8] /= norm;
//         x[9] /= norm;
//     }
//     else {

//         x[6] = 1.0f;
//         x[7] = 0.0f;
//         x[8] = 0.0f;
//         x[9] = 0.0f;
//     }
// }


// // ============================================================================
// // GET / SET QUATERNION
// // ============================================================================

// Quaternion OurEKF::get_state_quaternion() const
// {
//     Quaternion q;

//     q.q1 = x[6];
//     q.q2 = x[7];
//     q.q3 = x[8];
//     q.q4 = x[9];

//     return q;
// }


// void OurEKF::set_state_quaternion(
//     const Quaternion &q)
// {
//     x[6] = q.q1;
//     x[7] = q.q2;
//     x[8] = q.q3;
//     x[9] = q.q4;
// }


// // ============================================================================
// // GETTERS
// // ============================================================================

// Vector3f OurEKF::get_position() const
// {
//     return Vector3f(
//         x[0],
//         x[1],
//         x[2]);
// }


// Vector3f OurEKF::get_velocity() const
// {
//     return Vector3f(
//         x[3],
//         x[4],
//         x[5]);
// }


// Quaternion OurEKF::get_quaternion() const
// {
//     return get_state_quaternion();
// }


// Vector3f OurEKF::get_accel_bias() const
// {
//     return Vector3f(
//         x[10],
//         x[11],
//         x[12]);
// }


// Vector3f OurEKF::get_gyro_bias() const
// {
//     return Vector3f(
//         x[13],
//         x[14],
//         x[15]);
// }


// Vector3f OurEKF::get_accel_body() const
// {
//     return last_accel_body;
// }


// Vector3f OurEKF::get_accel_world() const
// {
//     return last_accel_world;
// }


// // ============================================================================
// // INDIVIDUAL GETTERS
// // ============================================================================

// float OurEKF::get_position_x() const
// {
//     return x[0];
// }

// float OurEKF::get_position_y() const
// {
//     return x[1];
// }

// float OurEKF::get_position_z() const
// {
//     return x[2];
// }

// float OurEKF::get_velocity_x() const
// {
//     return x[3];
// }

// float OurEKF::get_velocity_y() const
// {
//     return x[4];
// }

// float OurEKF::get_velocity_z() const
// {
//     return x[5];
// }

// float OurEKF::get_accel_bias_x() const
// {
//     return x[10];
// }

// float OurEKF::get_accel_bias_y() const
// {
//     return x[11];
// }

// float OurEKF::get_accel_bias_z() const
// {
//     return x[12];
// }

// float OurEKF::get_gyro_bias_x() const
// {
//     return x[13];
// }

// float OurEKF::get_gyro_bias_y() const
// {
//     return x[14];
// }

// float OurEKF::get_gyro_bias_z() const
// {
//     return x[15];
// }

// float OurEKF::get_roll() const
// {
//     return roll;
// }

// float OurEKF::get_pitch() const
// {
//     return pitch;
// }

// float OurEKF::get_yaw() const
// {
//     return yaw;
// }


// // ============================================================================
// // COMPLETE STATE
// // ============================================================================

// void OurEKF::get_state(
//     float state[STATE_SIZE]) const
// {
//     for (int i = 0; i < STATE_SIZE; i++) {
//         state[i] = x[i];
//     }
// }


// // ============================================================================
// // COMPLETE COVARIANCE
// // ============================================================================

// void OurEKF::get_covariance(
//     float covariance[STATE_SIZE][STATE_SIZE]) const
// {
//     for (int i = 0; i < STATE_SIZE; i++) {

//         for (int j = 0; j < STATE_SIZE; j++) {

//             covariance[i][j] =
//                 P[i][j];
//         }
//     }
// }


// // ============================================================================
// // INNOVATION GETTERS
// // ============================================================================

// float OurEKF::get_innovation_x() const
// {
//     return innovation[0];
// }

// float OurEKF::get_innovation_y() const
// {
//     return innovation[1];
// }

// float OurEKF::get_innovation_z() const
// {
//     return innovation[2];
// }


// // ============================================================================
// // INITIALIZED
// // ============================================================================

// bool OurEKF::is_initialized() const
// {
//     return initialized;
// }


// // ============================================================================
// // MATRIX HELPERS
// // ============================================================================

// void OurEKF::zero_matrix(
//     float A[STATE_SIZE][STATE_SIZE])
// {
//     memset(
//         A,
//         0,
//         sizeof(float) *
//         STATE_SIZE *
//         STATE_SIZE);
// }


// void OurEKF::zero_matrix_x3(
//     float A[STATE_SIZE][3])
// {
//     memset(
//         A,
//         0,
//         sizeof(float) *
//         STATE_SIZE *
//         3);
// }


// void OurEKF::zero_matrix_3x(
//     float A[3][STATE_SIZE])
// {
//     memset(
//         A,
//         0,
//         sizeof(float) *
//         3 *
//         STATE_SIZE);
// }


// void OurEKF::zero_matrix_3(
//     float A[3][3])
// {
//     memset(
//         A,
//         0,
//         sizeof(float) *
//         3 *
//         3);
// }


// void OurEKF::identity_matrix(
//     float A[STATE_SIZE][STATE_SIZE])
// {
//     zero_matrix(A);

//     for (int i = 0; i < STATE_SIZE; i++) {
//         A[i][i] = 1.0f;
//     }
// }


// // ============================================================================
// // 3x3 INVERSE
// // ============================================================================

// bool OurEKF::inverse_3x3(
//     const float A[3][3],
//     float A_inv[3][3]) const
// {
//     const float det =
//           A[0][0] *
//           (A[1][1] * A[2][2] -
//            A[1][2] * A[2][1])

//         - A[0][1] *
//           (A[1][0] * A[2][2] -
//            A[1][2] * A[2][0])

//         + A[0][2] *
//           (A[1][0] * A[2][1] -
//            A[1][1] * A[2][0]);

//     if (fabsf(det) < 1.0e-9f) {
//         return false;
//     }

//     const float inv_det =
//         1.0f / det;

//     A_inv[0][0] =
//         (A[1][1] * A[2][2] -
//          A[1][2] * A[2][1]) *
//         inv_det;

//     A_inv[0][1] =
//         (A[0][2] * A[2][1] -
//          A[0][1] * A[2][2]) *
//         inv_det;

//     A_inv[0][2] =
//         (A[0][1] * A[1][2] -
//          A[0][2] * A[1][1]) *
//         inv_det;

//     A_inv[1][0] =
//         (A[1][2] * A[2][0] -
//          A[1][0] * A[2][2]) *
//         inv_det;

//     A_inv[1][1] =
//         (A[0][0] * A[2][2] -
//          A[0][2] * A[2][0]) *
//         inv_det;

//     A_inv[1][2] =
//         (A[0][2] * A[1][0] -
//          A[0][0] * A[1][2]) *
//         inv_det;

//     A_inv[2][0] =
//         (A[1][0] * A[2][1] -
//          A[1][1] * A[2][0]) *
//         inv_det;

//     A_inv[2][1] =
//         (A[0][1] * A[2][0] -
//          A[0][0] * A[2][1]) *
//         inv_det;

//     A_inv[2][2] =
//         (A[0][0] * A[1][1] -
//          A[0][1] * A[1][0]) *
//         inv_det;

//     return true;
// }



#include "ownEkf.h"

#include <string.h>
#include <math.h>

#include <AP_HAL/AP_HAL.h>

extern const AP_HAL::HAL& hal;

// ============================================================================
// Tuning constants
// ============================================================================
namespace {

constexpr float GRAVITY = 9.80665f;

// Process noise
constexpr float ACCEL_NOISE_STD      = 0.20f;    // m/s^2
constexpr float GYRO_NOISE_STD       = 0.02f;    // rad/s
constexpr float ACCEL_BIAS_RW_STD    = 0.001f;
constexpr float GYRO_BIAS_RW_STD     = 0.0005f;
constexpr float EARTH_MAG_RW_STD     = 0.001f;
constexpr float BODY_MAG_RW_STD      = 0.001f;

// GPS
constexpr float GPS_POS_STD_NE       = 2.0f;     // m
constexpr float GPS_POS_STD_D        = 4.0f;     // m
constexpr float GPS_VEL_STD          = 0.5f;     // m/s
constexpr float GPS_GATE_SIGMA       = 5.0f;

// Magnetometer
constexpr float MAG_GATE_SIGMA       = 5.0f;
constexpr float BODY_MAG_INIT_VAR    = 1.0f;

// Accelerometer tilt aiding
constexpr float TILT_OBS_STD         = 0.35f;    // m/s^2
constexpr float TILT_GATE_SIGMA      = 4.0f;
constexpr float TILT_NORM_MIN        = 8.0f;
constexpr float TILT_NORM_MAX        = 11.5f;

// State limits
constexpr float ACCEL_BIAS_LIMIT     = 1.0f;     // m/s^2
constexpr float GYRO_BIAS_LIMIT      = 0.2f;     // rad/s

// d(R_bn * a)/dq  (body -> NED applied to vector a), columns w x y z
void jac_body_to_ned(const Quaternion &q, const Vector3f &a, float J[3][4])
{
    const float w = q.q1, x = q.q2, y = q.q3, z = q.q4;
    const float ax = a.x, ay = a.y, az = a.z;

    J[0][0] = 2.0f * (y * az - z * ay);
    J[0][1] = 2.0f * (y * ay + z * az);
    J[0][2] = -4.0f * y * ax + 2.0f * x * ay + 2.0f * w * az;
    J[0][3] = -4.0f * z * ax - 2.0f * w * ay + 2.0f * x * az;

    J[1][0] = 2.0f * (z * ax - x * az);
    J[1][1] = 2.0f * y * ax - 4.0f * x * ay - 2.0f * w * az;
    J[1][2] = 2.0f * x * ax + 2.0f * z * az;
    J[1][3] = 2.0f * w * ax - 4.0f * z * ay + 2.0f * y * az;

    J[2][0] = 2.0f * (x * ay - y * ax);
    J[2][1] = 2.0f * z * ax + 2.0f * w * ay - 4.0f * x * az;
    J[2][2] = -2.0f * w * ax + 2.0f * z * ay - 4.0f * y * az;
    J[2][3] = 2.0f * x * ax + 2.0f * y * ay;
}

// d(R_nb * v)/dq  (NED -> body applied to vector v), columns w x y z
void jac_ned_to_body(const Quaternion &q, const Vector3f &v, float J[3][4])
{
    const float w = q.q1, x = q.q2, y = q.q3, z = q.q4;
    const float vx = v.x, vy = v.y, vz = v.z;

    J[0][0] = 2.0f * (z * vy - y * vz);
    J[0][1] = 2.0f * (y * vy + z * vz);
    J[0][2] = -4.0f * y * vx + 2.0f * x * vy - 2.0f * w * vz;
    J[0][3] = -4.0f * z * vx + 2.0f * w * vy + 2.0f * x * vz;

    J[1][0] = 2.0f * (x * vz - z * vx);
    J[1][1] = 2.0f * y * vx - 4.0f * x * vy + 2.0f * w * vz;
    J[1][2] = 2.0f * x * vx + 2.0f * z * vz;
    J[1][3] = -2.0f * w * vx - 4.0f * z * vy + 2.0f * y * vz;

    J[2][0] = 2.0f * (y * vx - x * vy);
    J[2][1] = 2.0f * z * vx - 2.0f * w * vy - 4.0f * x * vz;
    J[2][2] = 2.0f * w * vx + 2.0f * z * vy - 4.0f * y * vz;
    J[2][3] = 2.0f * x * vx + 2.0f * y * vy;
}

} // namespace


// ============================================================================
// Constructor
// ============================================================================

OurEKF::OurEKF()
{
    memset(x, 0, sizeof(x));
    memset(P, 0, sizeof(P));
    memset(F, 0, sizeof(F));
    memset(Q, 0, sizeof(Q));

    memset(K, 0, sizeof(K));
    memset(PHt, 0, sizeof(PHt));
    memset(innovation, 0, sizeof(innovation));
    memset(innovation_vel, 0, sizeof(innovation_vel));
    memset(innovation_mag, 0, sizeof(innovation_mag));

    memset(cov_temp1, 0, sizeof(cov_temp1));
    memset(cov_temp2, 0, sizeof(cov_temp2));

    roll  = 0.0f;
    pitch = 0.0f;
    yaw   = 0.0f;

    initialized = false;
    healthy = true;
    reset_count = 0;

    have_last_gps = false;
    last_gps_position = Vector3f(0.0f, 0.0f, 0.0f);

    mag_learning = false;
    tilt_fusion_enabled = true;
    mag_noise_std = 0.05f;

    have_earth_seed = false;
    earth_seed = Vector3f(0.0f, 0.0f, 0.0f);
    earth_seed_std = 1.0f;

    last_accel_body  = Vector3f(0.0f, 0.0f, 0.0f);
    last_accel_world = Vector3f(0.0f, 0.0f, 0.0f);
}


// ============================================================================
// INIT
// ============================================================================

void OurEKF::init(const Vector3f &position,
                  const Vector3f &velocity,
                  const Quaternion &quaternion)
{
    memset(x, 0, sizeof(x));
    memset(P, 0, sizeof(P));

    x[0] = position.x;
    x[1] = position.y;
    x[2] = position.z;

    x[3] = velocity.x;
    x[4] = velocity.y;
    x[5] = velocity.z;

    x[6] = quaternion.q1;
    x[7] = quaternion.q2;
    x[8] = quaternion.q3;
    x[9] = quaternion.q4;

    // Biases and magnetic states start at zero.

    have_last_gps = false;
    memset(innovation, 0, sizeof(innovation));
    memset(innovation_vel, 0, sizeof(innovation_vel));
    memset(innovation_mag, 0, sizeof(innovation_mag));
    last_accel_body  = Vector3f(0.0f, 0.0f, 0.0f);
    last_accel_world = Vector3f(0.0f, 0.0f, 0.0f);

    normalize_quaternion();

    // Initial covariance
    P[0][0] = P[1][1] = P[2][2] = 1.0f;          // position
    P[3][3] = P[4][4] = P[5][5] = 0.5f;          // velocity
    P[6][6] = P[7][7] = P[8][8] = P[9][9] = 0.01f;   // quaternion
    P[10][10] = P[11][11] = P[12][12] = 0.01f;   // accel bias
    P[13][13] = P[14][14] = P[15][15] = 0.01f;   // gyro bias
    P[16][16] = P[17][17] = P[18][18] = 1.0f;    // earth field

    // Body field: only learned when explicitly enabled.
    const float body_var = mag_learning ? BODY_MAG_INIT_VAR : 0.0f;
    P[19][19] = P[20][20] = P[21][21] = body_var;

    apply_earth_seed();

    Orientation();

    healthy = true;
    initialized = true;
}


// ============================================================================
// EARTH FIELD SEED
// ============================================================================

void OurEKF::init_earth_field(const Vector3f &earth_field_ned, float std_dev)
{
    earth_seed = earth_field_ned;
    earth_seed_std = std_dev;
    have_earth_seed = true;

    if (initialized) {
        apply_earth_seed();
    }
}

void OurEKF::apply_earth_seed()
{
    if (!have_earth_seed) {
        return;
    }

    x[16] = earth_seed.x;
    x[17] = earth_seed.y;
    x[18] = earth_seed.z;

    for (int i = 16; i <= 18; i++) {
        for (int j = 0; j < STATE_SIZE; j++) {
            P[i][j] = 0.0f;
            P[j][i] = 0.0f;
        }
        P[i][i] = earth_seed_std * earth_seed_std;
    }
}

void OurEKF::set_body_mag_learning(bool enable)
{
    if (enable == mag_learning) {
        return;
    }

    mag_learning = enable;

    for (int i = 19; i <= 21; i++) {
        for (int j = 0; j < STATE_SIZE; j++) {
            P[i][j] = 0.0f;
            P[j][i] = 0.0f;
        }
        P[i][i] = enable ? BODY_MAG_INIT_VAR : 0.0f;
    }
}


// ============================================================================
// INITIAL ATTITUDE FROM ACCEL + MAG
// ============================================================================

Quaternion OurEKF::attitude_from_accel_mag(const Vector3f &accel,
                                           const Vector3f &mag,
                                           float declination_rad)
{
    // accel is specific force (about -g in body frame when static)
    const float r = atan2f(-accel.y, -accel.z);
    const float p = atan2f(accel.x, sqrtf(accel.y * accel.y + accel.z * accel.z));

    const float sr = sinf(r), cr = cosf(r);
    const float sp = sinf(p), cp = cosf(p);

    const float hx = mag.x * cp + mag.y * sr * sp + mag.z * cr * sp;
    const float hy = mag.y * cr - mag.z * sr;

    float y = atan2f(-hy, hx) + declination_rad;
    if (y > M_PI)  { y -= 2.0f * M_PI; }
    if (y < -M_PI) { y += 2.0f * M_PI; }

    Quaternion q;
    q.from_euler(r, p, y);
    return q;
}


// ============================================================================
// COMPLETE UPDATE
// ============================================================================

void OurEKF::update(const Vector3f &accel,
                    const Vector3f &gyro,
                    const Vector3f &mag,
                    float dt,
                    bool gps_available,
                    const Vector3f &gps_position)
{
    update(accel, gyro, mag, dt,
           gps_available, gps_position,
           false, Vector3f(0.0f, 0.0f, 0.0f));
}

void OurEKF::update(const Vector3f &accel,
                    const Vector3f &gyro,
                    const Vector3f &mag,
                    float dt,
                    bool gps_available,
                    const Vector3f &gps_position,
                    bool gps_vel_available,
                    const Vector3f &gps_velocity)
{
    if (!initialized) {
        return;
    }

    // Prediction (+ optional tilt aiding)
    predict(accel, gyro, dt);

    // GPS position, fused only when the sample looks new.
    // (Better: pass a GPS timestamp / new-sample flag from the caller.)
    if (gps_available) {
        bool new_gps = false;

        if (!have_last_gps) {
            new_gps = true;
        } else if ((gps_position - last_gps_position).length() > 0.001f) {
            new_gps = true;
        }

        if (new_gps) {
            update_gps(gps_position);
        }
    }

    if (gps_vel_available) {
        update_gps_velocity(gps_velocity);
    }

    update_mag(mag);

    if (!state_is_finite()) {
        reset_after_divergence();
    }
}


// ============================================================================
// PREDICTION
// ============================================================================

void OurEKF::predict(const Vector3f &accel,
                     const Vector3f &gyro,
                     float dt)
{
    if (!initialized) {
        return;
    }

    if (!(dt > 0.0f) || dt > 0.1f) {
        return;
    }

    // IMU bias correction
    Vector3f accel_corrected;
    accel_corrected.x = accel.x - x[10];
    accel_corrected.y = accel.y - x[11];
    accel_corrected.z = accel.z - x[12];

    Vector3f gyro_corrected;
    gyro_corrected.x = gyro.x - x[13];
    gyro_corrected.y = gyro.y - x[14];
    gyro_corrected.z = gyro.z - x[15];

    last_accel_body = accel_corrected;

    // Attitude at the start of the step (used for velocity and for F).
    const Quaternion q_old = get_state_quaternion();
    const Matrix3f Rnb = quaternion_to_rotation_matrix(q_old);   // NED -> body

    // Body -> NED is the transpose of Rnb.
    Vector3f accel_ned;
    accel_ned.x = Rnb.a.x * accel_corrected.x +
                  Rnb.b.x * accel_corrected.y +
                  Rnb.c.x * accel_corrected.z;
    accel_ned.y = Rnb.a.y * accel_corrected.x +
                  Rnb.b.y * accel_corrected.y +
                  Rnb.c.y * accel_corrected.z;
    accel_ned.z = Rnb.a.z * accel_corrected.x +
                  Rnb.b.z * accel_corrected.y +
                  Rnb.c.z * accel_corrected.z;

    // NED: gravity is +Z, IMU measures specific force.
    accel_ned.z += GRAVITY;

    last_accel_world = accel_ned;

    // Position prediction
    const float half_dt2 = 0.5f * dt * dt;
    x[0] += x[3] * dt + accel_ned.x * half_dt2;
    x[1] += x[4] * dt + accel_ned.y * half_dt2;
    x[2] += x[5] * dt + accel_ned.z * half_dt2;

    //velocity prediction
    x[3] += accel_ned.x * dt;
    x[4] += accel_ned.y * dt;
    x[5] += accel_ned.z * dt;

    // Covariance prediction (F evaluated at the pre-propagation quaternion)
    build_F(accel_corrected, gyro_corrected, dt);
    build_Q(dt);
    predict_covariance();

    // Quaternion prediction
    propagate_quaternion(gyro_corrected, dt);

    constrain_states();

    // Gravity aiding, only near 1 g so manoeuvres are not treated as gravity.
    if (tilt_fusion_enabled) {
        const float acc_norm = accel_corrected.length();
        if (acc_norm > TILT_NORM_MIN && acc_norm < TILT_NORM_MAX) {
            fuse_accel_tilt(accel);
        }
    }

    Orientation();
}


// ============================================================================
// ACCELEROMETER TILT AIDING
//
// z    = raw accel (specific force, body)
// zhat = R_nb * [0 0 -g] + accel_bias
// ============================================================================

void OurEKF::fuse_accel_tilt(const Vector3f &accel_raw)
{
    const float meas[3] = { accel_raw.x, accel_raw.y, accel_raw.z };
    const Vector3f gvec(0.0f, 0.0f, -GRAVITY);
    const float r_var = TILT_OBS_STD * TILT_OBS_STD;

    for (int axis = 0; axis < 3; axis++) {

        const Quaternion q = get_state_quaternion();
        const Matrix3f Rnb = quaternion_to_rotation_matrix(q);

        const Vector3f &row = (axis == 0) ? Rnb.a :
                              (axis == 1) ? Rnb.b :
                                            Rnb.c;

        const float zhat = row.z * (-GRAVITY) + x[10 + axis];
        const float innov = meas[axis] - zhat;

        float J[3][4];
        jac_ned_to_body(q, gvec, J);

        float Hrow[STATE_SIZE];
        for (int i = 0; i < STATE_SIZE; i++) {
            Hrow[i] = 0.0f;
        }
        for (int j = 0; j < 4; j++) {
            Hrow[6 + j] = J[axis][j];
        }
        Hrow[10 + axis] = 1.0f;

        fuse_scalar(Hrow, innov, r_var, TILT_GATE_SIGMA);
    }
}


// ============================================================================
// GENERIC SCALAR FUSION
//
// S = H P H' + R
// K = P H' / S
// x = x + K * innov
// P = P - K (P H')'      (P symmetric)
// ============================================================================

OurEKF::FuseResult OurEKF::fuse_scalar(const float *Hrow,
                                       float innov,
                                       float r_var,
                                       float gate_sigma)
{
    float S_val = r_var;

    for (int i = 0; i < STATE_SIZE; i++) {
        float s = 0.0f;
        for (int j = 0; j < STATE_SIZE; j++) {
            if (Hrow[j] != 0.0f) {
                s += P[i][j] * Hrow[j];
            }
        }
        PHt[i][0] = s;
        S_val += Hrow[i] * s;
    }

    if (!(S_val > 1.0e-9f) || !isfinite(S_val) || !isfinite(innov)) {
        return FUSE_INVALID;
    }

    if (innov * innov > gate_sigma * gate_sigma * S_val) {
        return FUSE_REJECTED;
    }

    const float inv_S = 1.0f / S_val;

    for (int i = 0; i < STATE_SIZE; i++) {
        K[i][0] = PHt[i][0] * inv_S;
        x[i] += K[i][0] * innov;
    }

    for (int i = 0; i < STATE_SIZE; i++) {
        const float ki = K[i][0];
        for (int j = 0; j < STATE_SIZE; j++) {
            P[i][j] -= ki * PHt[j][0];
        }
    }

    normalize_quaternion();
    constrain_states();
    force_symmetry_and_constrain();

    return FUSE_OK;
}


// ============================================================================
// QUATERNION PROPAGATION
// ============================================================================

void OurEKF::propagate_quaternion(const Vector3f &gyro, float dt)
{
    const Quaternion q = get_state_quaternion();
    const Quaternion dq = quaternion_from_gyro(gyro, dt);

    Quaternion q_new;

    q_new.q1 = q.q1 * dq.q1 - q.q2 * dq.q2 - q.q3 * dq.q3 - q.q4 * dq.q4;
    q_new.q2 = q.q1 * dq.q2 + q.q2 * dq.q1 + q.q3 * dq.q4 - q.q4 * dq.q3;
    q_new.q3 = q.q1 * dq.q3 - q.q2 * dq.q4 + q.q3 * dq.q1 + q.q4 * dq.q2;
    q_new.q4 = q.q1 * dq.q4 + q.q2 * dq.q3 - q.q3 * dq.q2 + q.q4 * dq.q1;

    set_state_quaternion(q_new);
    normalize_quaternion();
}


// ============================================================================
// DELTA QUATERNION
// ============================================================================

Quaternion OurEKF::quaternion_from_gyro(const Vector3f &gyro, float dt) const
{
    const float angle_x = gyro.x * dt;
    const float angle_y = gyro.y * dt;
    const float angle_z = gyro.z * dt;

    const float angle = sqrtf(angle_x * angle_x +
                              angle_y * angle_y +
                              angle_z * angle_z);

    Quaternion dq;

    if (angle < 1.0e-8f) {
        dq.q1 = 1.0f;
        dq.q2 = 0.0f;
        dq.q3 = 0.0f;
        dq.q4 = 0.0f;
        return dq;
    }

    const float half_angle = 0.5f * angle;
    const float s = sinf(half_angle) / angle;

    dq.q1 = cosf(half_angle);
    dq.q2 = angle_x * s;
    dq.q3 = angle_y * s;
    dq.q4 = angle_z * s;

    return dq;
}


// ============================================================================
// ORIENTATION (Euler angles)
// ============================================================================

void OurEKF::Orientation()
{
    const Quaternion q = get_state_quaternion();

    const float qw = q.q1;
    const float qx = q.q2;
    const float qy = q.q3;
    const float qz = q.q4;

    roll = atan2f(2.0f * (qw * qx + qy * qz),
                  1.0f - 2.0f * (qx * qx + qy * qy));

    float sin_pitch = 2.0f * (qw * qy - qz * qx);
    if (sin_pitch > 1.0f)  { sin_pitch = 1.0f; }
    if (sin_pitch < -1.0f) { sin_pitch = -1.0f; }

    pitch = asinf(sin_pitch);

    yaw = atan2f(2.0f * (qw * qz + qx * qy),
                 1.0f - 2.0f * (qy * qy + qz * qz));
}


// ============================================================================
// QUATERNION -> ROTATION MATRIX  (NED -> body)
// ============================================================================

Matrix3f OurEKF::quaternion_to_rotation_matrix(const Quaternion &q) const
{
    Matrix3f Row;

    const float qw = q.q1;
    const float qx = q.q2;
    const float qy = q.q3;
    const float qz = q.q4;

    Row.a.x = 1.0f - 2.0f * (qy * qy + qz * qz);
    Row.a.y = 2.0f * (qx * qy + qz * qw);
    Row.a.z = 2.0f * (qx * qz - qy * qw);

    Row.b.x = 2.0f * (qx * qy - qz * qw);
    Row.b.y = 1.0f - 2.0f * (qx * qx + qz * qz);
    Row.b.z = 2.0f * (qy * qz + qx * qw);

    Row.c.x = 2.0f * (qx * qz + qy * qw);
    Row.c.y = 2.0f * (qy * qz - qx * qw);
    Row.c.z = 1.0f - 2.0f * (qx * qx + qy * qy);

    return Row;
}


// ============================================================================
// F MATRIX
//
// Uses the quaternion state at the START of the step (before propagation).
// ============================================================================

void OurEKF::build_F(const Vector3f &accel_body,
                     const Vector3f &gyro_body,
                     float dt)
{
    zero_matrix(F);

    for (int i = 0; i < STATE_SIZE; i++) {
        F[i][i] = 1.0f;
    }

    // Position <- velocity
    F[0][3] = dt;
    F[1][4] = dt;
    F[2][5] = dt;

    const Quaternion q = get_state_quaternion();
    const Matrix3f Rnb = quaternion_to_rotation_matrix(q);

    // Body -> NED matrix Rbn[i][j] = Rnb[j][i]
    const float Rbn[3][3] = {
        { Rnb.a.x, Rnb.b.x, Rnb.c.x },
        { Rnb.a.y, Rnb.b.y, Rnb.c.y },
        { Rnb.a.z, Rnb.b.z, Rnb.c.z }
    };

    const float half_dt2 = 0.5f * dt * dt;

    // Position / velocity <- accel bias
    for (int i = 0; i < 3; i++) {
        for (int j = 0; j < 3; j++) {
            F[i][10 + j]     = -half_dt2 * Rbn[i][j];
            F[3 + i][10 + j] = -dt * Rbn[i][j];
        }
    }

    // Position / velocity <- quaternion  d(Rbn * a)/dq
    float J[3][4];
    jac_body_to_ned(q, accel_body, J);

    for (int i = 0; i < 3; i++) {
        for (int j = 0; j < 4; j++) {
            F[3 + i][6 + j] = dt * J[i][j];
            F[i][6 + j]     = half_dt2 * J[i][j];
        }
    }

    // Quaternion <- quaternion:  I + 0.5*dt*M(w),  q_dot = 0.5 q (x) (0,w)
    const float wx = gyro_body.x;
    const float wy = gyro_body.y;
    const float wz = gyro_body.z;
    const float h  = 0.5f * dt;

    const float M[4][4] = {
        { 0.0f, -wx,  -wy,  -wz  },
        { wx,   0.0f,  wz,  -wy  },
        { wy,  -wz,   0.0f,  wx  },
        { wz,   wy,   -wx,  0.0f }
    };

    for (int i = 0; i < 4; i++) {
        for (int j = 0; j < 4; j++) {
            F[6 + i][6 + j] += h * M[i][j];
        }
    }

    // Quaternion <- gyro bias
    const float qw = q.q1;
    const float qx = q.q2;
    const float qy = q.q3;
    const float qz = q.q4;

    F[6][13] =  h * qx;   F[6][14] =  h * qy;   F[6][15] =  h * qz;
    F[7][13] = -h * qw;   F[7][14] =  h * qz;   F[7][15] = -h * qy;
    F[8][13] = -h * qz;   F[8][14] = -h * qw;   F[8][15] =  h * qx;
    F[9][13] =  h * qy;   F[9][14] = -h * qx;   F[9][15] = -h * qw;

    // Magnetic field states: slowly varying, F diagonal = 1.
}


// ============================================================================
// Q MATRIX
// ============================================================================

void OurEKF::build_Q(float dt)
{
    zero_matrix(Q);

    const float accel_var      = ACCEL_NOISE_STD * ACCEL_NOISE_STD;
    const float gyro_var       = GYRO_NOISE_STD * GYRO_NOISE_STD;
    const float accel_bias_var = ACCEL_BIAS_RW_STD * ACCEL_BIAS_RW_STD;
    const float gyro_bias_var  = GYRO_BIAS_RW_STD * GYRO_BIAS_RW_STD;
    const float earth_mag_var  = EARTH_MAG_RW_STD * EARTH_MAG_RW_STD;
    const float body_mag_var   = BODY_MAG_RW_STD * BODY_MAG_RW_STD;

    const float dt2 = dt * dt;
    const float dt4 = dt2 * dt2;

    // Position
    const float position_noise = 0.25f * accel_var * dt4;
    Q[0][0] = Q[1][1] = Q[2][2] = position_noise;

    // Velocity
    const float velocity_noise = accel_var * dt2;
    Q[3][3] = Q[4][4] = Q[5][5] = velocity_noise;

    // Quaternion: 0.25 dt^2 sigma^2 (I - q q')  (rank 3, keeps |q| direction quiet)
    const float qs[4] = { x[6], x[7], x[8], x[9] };
    const float qn = 0.25f * gyro_var * dt2;

    for (int i = 0; i < 4; i++) {
        for (int j = 0; j < 4; j++) {
            Q[6 + i][6 + j] = qn * ((i == j ? 1.0f : 0.0f) - qs[i] * qs[j]);
        }
    }

    // Accelerometer bias
    Q[10][10] = Q[11][11] = Q[12][12] = accel_bias_var * dt;

    // Gyro bias
    Q[13][13] = Q[14][14] = Q[15][15] = gyro_bias_var * dt;

    // Earth magnetic field
    Q[16][16] = Q[17][17] = Q[18][18] = earth_mag_var * dt;

    // Body magnetic field (only while learning)
    if (mag_learning) {
        Q[19][19] = Q[20][20] = Q[21][21] = body_mag_var * dt;
    }
}


// ============================================================================
// COVARIANCE PREDICTION
//
// P = F P F' + Q
//
// F is sparse (identity plus a few blocks), so zero entries are skipped.
// cov_temp1 / cov_temp2 are class members (no big stack arrays).
// ============================================================================

void OurEKF::predict_covariance()
{
    // temp1 = F * P
    zero_matrix(cov_temp1);

    for (int i = 0; i < STATE_SIZE; i++) {
        for (int k = 0; k < STATE_SIZE; k++) {
            const float f = F[i][k];
            if (f == 0.0f) {
                continue;
            }
            for (int j = 0; j < STATE_SIZE; j++) {
                cov_temp1[i][j] += f * P[k][j];
            }
        }
    }

    // temp2 = temp1 * F'
    zero_matrix(cov_temp2);

    for (int j = 0; j < STATE_SIZE; j++) {
        for (int k = 0; k < STATE_SIZE; k++) {
            const float f = F[j][k];
            if (f == 0.0f) {
                continue;
            }
            for (int i = 0; i < STATE_SIZE; i++) {
                cov_temp2[i][j] += cov_temp1[i][k] * f;
            }
        }
    }

    // P = temp2 + Q
    for (int i = 0; i < STATE_SIZE; i++) {
        for (int j = 0; j < STATE_SIZE; j++) {
            P[i][j] = cov_temp2[i][j] + Q[i][j];
        }
    }

    force_symmetry_and_constrain();
}


// ============================================================================
// GPS POSITION UPDATE (sequential N, E, D)
// ============================================================================

bool OurEKF::update_gps(const Vector3f &gps_position)
{
    if (!initialized) {
        return false;
    }

    const float meas[3] = { gps_position.x, gps_position.y, gps_position.z };
    const float std_axis[3] = { GPS_POS_STD_NE, GPS_POS_STD_NE, GPS_POS_STD_D };

    for (int axis = 0; axis < 3; axis++) {

        const float innov = meas[axis] - x[axis];
        innovation[axis] = innov;

        float Hrow[STATE_SIZE];
        for (int i = 0; i < STATE_SIZE; i++) {
            Hrow[i] = 0.0f;
        }
        Hrow[axis] = 1.0f;

        const FuseResult res = fuse_scalar(Hrow,
                                           innov,
                                           std_axis[axis] * std_axis[axis],
                                           GPS_GATE_SIGMA);
        if (res == FUSE_INVALID) {
            return false;
        }
    }

    last_gps_position = gps_position;
    have_last_gps = true;

    Orientation();
    return true;
}


// ============================================================================
// GPS VELOCITY UPDATE (sequential N, E, D)
// ============================================================================

bool OurEKF::update_gps_velocity(const Vector3f &gps_velocity)
{
    if (!initialized) {
        return false;
    }

    const float meas[3] = { gps_velocity.x, gps_velocity.y, gps_velocity.z };
    const float r_var = GPS_VEL_STD * GPS_VEL_STD;

    for (int axis = 0; axis < 3; axis++) {

        const float innov = meas[axis] - x[3 + axis];
        innovation_vel[axis] = innov;

        float Hrow[STATE_SIZE];
        for (int i = 0; i < STATE_SIZE; i++) {
            Hrow[i] = 0.0f;
        }
        Hrow[3 + axis] = 1.0f;

        const FuseResult res = fuse_scalar(Hrow, innov, r_var, GPS_GATE_SIGMA);
        if (res == FUSE_INVALID) {
            return false;
        }
    }

    Orientation();
    return true;
}


// ============================================================================
// MAGNETOMETER PREDICTION
//
// predicted_body_mag = R_nb * earth_mag + body_mag
// ============================================================================

Vector3f OurEKF::predict_magnetometer(const Quaternion &q) const
{
    const Matrix3f Row = quaternion_to_rotation_matrix(q);

    const float ex = x[16];
    const float ey = x[17];
    const float ez = x[18];

    Vector3f predicted;

    predicted.x = Row.a.x * ex + Row.a.y * ey + Row.a.z * ez + x[19];
    predicted.y = Row.b.x * ex + Row.b.y * ey + Row.b.z * ez + x[20];
    predicted.z = Row.c.x * ex + Row.c.y * ey + Row.c.z * ez + x[21];

    return predicted;
}


// ============================================================================
// MAGNETOMETER UPDATE
//
// Three components fused sequentially with an analytical Jacobian.
// ============================================================================

/*bool OurEKF::update_mag(const Vector3f &mag)
{
    if (!initialized) {
        return false;
    }

    const float mag_norm = sqrtf(mag.x * mag.x + mag.y * mag.y + mag.z * mag.z);

    if (!(mag_norm > 1.0e-4f) || !isfinite(mag_norm)) {
        return false;
    }

    // If the earth field was never seeded (e.g. from WMM), take it from the
    // first valid compass sample. Yaw is then only relative to the initial
    // quaternion.
    const float earth_norm =
        sqrtf(x[16] * x[16] + x[17] * x[17] + x[18] * x[18]);

    if (earth_norm < 1.0e-4f) {
        const Matrix3f R0 = quaternion_to_rotation_matrix(get_state_quaternion());
        x[16] = R0.a.x * mag.x + R0.b.x * mag.y + R0.c.x * mag.z;
        x[17] = R0.a.y * mag.x + R0.b.y * mag.y + R0.c.y * mag.z;
        x[18] = R0.a.z * mag.x + R0.b.z * mag.y + R0.c.z * mag.z;
    }

    const float meas[3] = { mag.x, mag.y, mag.z };
    const float r_var = mag_noise_std * mag_noise_std;

    for (int comp = 0; comp < 3; comp++) {

        // Recompute after each component: previous fusion changed the state.
        const Quaternion q = get_state_quaternion();
        const Vector3f predicted = predict_magnetometer(q);
        const Matrix3f Row = quaternion_to_rotation_matrix(q);

        const float zhat = (comp == 0) ? predicted.x :
                           (comp == 1) ? predicted.y :
                                         predicted.z;

        const float innov = meas[comp] - zhat;
        innovation_mag[comp] = innov;

        float J[3][4];
        jac_ned_to_body(q, Vector3f(x[16], x[17], x[18]), J);

        float Hrow[STATE_SIZE];
        for (int i = 0; i < STATE_SIZE; i++) {
            Hrow[i] = 0.0f;
        }

        // Quaternion
        for (int j = 0; j < 4; j++) {
            Hrow[6 + j] = J[comp][j];
        }

        // Earth field: row of R_nb
        const Vector3f &row = (comp == 0) ? Row.a :
                              (comp == 1) ? Row.b :
                                            Row.c;
        Hrow[16] = row.x;
        Hrow[17] = row.y;
        Hrow[18] = row.z;

        // Body field
        if (mag_learning) {
            Hrow[19 + comp] = 1.0f;
        }

        const FuseResult res = fuse_scalar(Hrow, innov, r_var, MAG_GATE_SIGMA);
        if (res == FUSE_INVALID) {
            return false;
        }
    }

    Orientation();
    return true;
}*/

bool OurEKF::update_mag(const Vector3f &mag)
{
    if (!initialized) {
        return false;
    }

    // ------------------------------------------------------------
    // 1. Check magnetometer measurement
    // ------------------------------------------------------------

    const float mag_norm =
        sqrtf(mag.x * mag.x +
              mag.y * mag.y +
              mag.z * mag.z);

    if (!isfinite(mag_norm) || mag_norm < 1.0e-4f) {
        return false;
    }

    // ------------------------------------------------------------
    // 2. Get current EKF quaternion
    // ------------------------------------------------------------

    Quaternion q = get_state_quaternion();

    float qw = q.q1;
    float qx = q.q2;
    float qy = q.q3;
    float qz = q.q4;

    // ------------------------------------------------------------
    // 3. Get EKF roll and pitch
    //
    // We use roll/pitch from the EKF only for tilt compensation.
    // Yaw itself is NOT used to calculate the magnetometer yaw.
    // ------------------------------------------------------------

    const float sinr_cosp =
        2.0f * (qw * qx + qy * qz);

    const float cosr_cosp =
        1.0f - 2.0f * (qx * qx + qy * qy);

    const float rolll =
        atan2f(sinr_cosp, cosr_cosp);

    const float sinp =
        2.0f * (qw * qy - qz * qx);

    float pitchh;

    if (fabsf(sinp) >= 1.0f) {
        pitchh = copysignf(0.5f * M_PI, sinp);
    } else {
        pitchh = asinf(sinp);
    }

    // ------------------------------------------------------------
    // 4. Tilt compensate magnetometer
    //
    // This removes the effect of roll/pitch from the magnetic
    // measurement before calculating heading.
    // ------------------------------------------------------------

    const float cr = cosf(rolll);
    const float sr = sinf(rolll);
    const float cp = cosf(pitchh);
    const float sp = sinf(pitchh);

    const float mag_x_horizontal =
        mag.x * cp +
        mag.z * sp;

    const float mag_y_horizontal =
        mag.x * sr * sp +
        mag.y * cr -
        mag.z * sr * cp;

    // ------------------------------------------------------------
    // 5. Calculate magnetic yaw measurement
    // ------------------------------------------------------------

    float mag_yaw =
        atan2f(-mag_y_horizontal,
                mag_x_horizontal);

    if (!isfinite(mag_yaw)) {
        return false;
    }

    // ------------------------------------------------------------
    // 6. Calculate current EKF yaw
    // ------------------------------------------------------------

    const float yaw_num =
        2.0f * (qw * qz + qx * qy);

    const float yaw_den =
        1.0f - 2.0f * (qy * qy + qz * qz);

    const float ekf_yaw =
        atan2f(yaw_num, yaw_den);

    // ------------------------------------------------------------
    // 7. Calculate yaw innovation
    // ------------------------------------------------------------

    float yaw_error =
        mag_yaw - ekf_yaw;

    // Wrap innovation to [-PI, +PI]
    while (yaw_error > M_PI) {
        yaw_error -= 2.0f * M_PI;
    }

    while (yaw_error < -M_PI) {
        yaw_error += 2.0f * M_PI;
    }

    innovation_mag[0] = yaw_error;

    // ------------------------------------------------------------
    // 8. Build H
    //
    // Measurement:
    //
    //     z = yaw(q)
    //
    // Therefore:
    //
    //     H = d(yaw)/d(x)
    //
    // Only quaternion states are relevant.
    //
    // x[6] = qw
    // x[7] = qx
    // x[8] = qy
    // x[9] = qz
    // ------------------------------------------------------------

    float Hrow[STATE_SIZE];

    for (int i = 0; i < STATE_SIZE; i++) {
        Hrow[i] = 0.0f;
    }

    const float denom =
        yaw_num * yaw_num +
        yaw_den * yaw_den;

    if (denom < 1.0e-8f) {
        return false;
    }

    // yaw = atan2(N,D)
    //
    // d yaw = (D*dN - N*dD)/(N²+D²)

    const float dN_dqw = 2.0f * qz;
    const float dN_dqx = 2.0f * qy;
    const float dN_dqy = 2.0f * qx;
    const float dN_dqz = 2.0f * qw;

    const float dD_dqw = 0.0f;
    const float dD_dqx = 0.0f;
    const float dD_dqy = -4.0f * qy;
    const float dD_dqz = -4.0f * qz;

    Hrow[6] =
        (yaw_den * dN_dqw -
         yaw_num * dD_dqw) / denom;

    Hrow[7] =
        (yaw_den * dN_dqx -
         yaw_num * dD_dqx) / denom;

    Hrow[8] =
        (yaw_den * dN_dqy -
         yaw_num * dD_dqy) / denom;

    Hrow[9] =
        (yaw_den * dN_dqz -
         yaw_num * dD_dqz) / denom;

    // ------------------------------------------------------------
    // 9. Magnetometer measurement noise
    //
    // 5 degrees standard deviation
    // ------------------------------------------------------------

    const float mag_yaw_std =
        5.0f * DEG_TO_RAD;

    const float mag_yaw_var =
        mag_yaw_std * mag_yaw_std;

    // ------------------------------------------------------------
    // 10. Fuse scalar yaw measurement
    //
    // fuse_scalar() performs:
    //
    //     S = HPH' + R
    //     K = PH'/S
    //     x = x + K*innovation
    //     P = P - K*PH'
    //
    // It also normalizes the quaternion afterwards.
    // ------------------------------------------------------------

    const FuseResult result =
        fuse_scalar(
            Hrow,
            yaw_error,
            mag_yaw_var,
            MAG_GATE_SIGMA);

    if (result == FUSE_INVALID) {
        return false;
    }

    // ------------------------------------------------------------
    // 11. Update orientation output
    // ------------------------------------------------------------

    Orientation();

    return true;
}


// ============================================================================
// HOUSEKEEPING
// ============================================================================

void OurEKF::force_symmetry_and_constrain()
{
    for (int i = 0; i < STATE_SIZE; i++) {

        if (!(P[i][i] > 1.0e-12f)) {
            P[i][i] = 1.0e-12f;
        }

        for (int j = i + 1; j < STATE_SIZE; j++) {
            const float v = 0.5f * (P[i][j] + P[j][i]);
            P[i][j] = v;
            P[j][i] = v;
        }
    }
}

void OurEKF::constrain_states()
{
    for (int i = 10; i <= 12; i++) {
        if (x[i] > ACCEL_BIAS_LIMIT)  { x[i] = ACCEL_BIAS_LIMIT; }
        if (x[i] < -ACCEL_BIAS_LIMIT) { x[i] = -ACCEL_BIAS_LIMIT; }
    }

    for (int i = 13; i <= 15; i++) {
        if (x[i] > GYRO_BIAS_LIMIT)  { x[i] = GYRO_BIAS_LIMIT; }
        if (x[i] < -GYRO_BIAS_LIMIT) { x[i] = -GYRO_BIAS_LIMIT; }
    }
}

bool OurEKF::state_is_finite() const
{
    for (int i = 0; i < STATE_SIZE; i++) {
        if (!isfinite(x[i]) || !isfinite(P[i][i])) {
            return false;
        }
    }
    return true;
}

// Re-initialise after NaN/Inf. Attitude is reset to level (identity):
// tilt aiding recovers roll/pitch; yaw is relative until the earth field is
// re-seeded. The caller can watch get_reset_count() / is_healthy() and
// re-initialise with a better attitude if needed.
void OurEKF::reset_after_divergence()
{
    const Vector3f pos = have_last_gps ? last_gps_position
                                       : Vector3f(0.0f, 0.0f, 0.0f);

    init(pos, Vector3f(0.0f, 0.0f, 0.0f), Quaternion(1.0f, 0.0f, 0.0f, 0.0f));

    reset_count++;
    healthy = false;
}



// QUATERNION NORMALIZATION


void OurEKF::normalize_quaternion()
{
    const float qw = x[6];
    const float qx = x[7];
    const float qy = x[8];
    const float qz = x[9];

    const float norm = sqrtf(qw * qw + qx * qx + qy * qy + qz * qz);

    if (norm > 1.0e-6f && isfinite(norm)) {
        x[6] /= norm;
        x[7] /= norm;
        x[8] /= norm;
        x[9] /= norm;
    } else {
        x[6] = 1.0f;
        x[7] = 0.0f;
        x[8] = 0.0f;
        x[9] = 0.0f;
    }
}



// GET / SET QUATERNION


Quaternion OurEKF::get_state_quaternion() const
{
    Quaternion q;

    q.q1 = x[6];
    q.q2 = x[7];
    q.q3 = x[8];
    q.q4 = x[9];

    return q;
}

void OurEKF::set_state_quaternion(const Quaternion &q)
{
    x[6] = q.q1;
    x[7] = q.q2;
    x[8] = q.q3;
    x[9] = q.q4;
}


// ============================================================================
// GETTERS
// ============================================================================

Vector3f OurEKF::get_position() const     { return Vector3f(x[0], x[1], x[2]); }
Vector3f OurEKF::get_velocity() const     { return Vector3f(x[3], x[4], x[5]); }
Quaternion OurEKF::get_quaternion() const { return get_state_quaternion(); }

Vector3f OurEKF::get_accel_bias() const   { return Vector3f(x[10], x[11], x[12]); }
Vector3f OurEKF::get_gyro_bias() const    { return Vector3f(x[13], x[14], x[15]); }

Vector3f OurEKF::get_accel_body() const   { return last_accel_body; }
Vector3f OurEKF::get_accel_world() const  { return last_accel_world; }

float OurEKF::get_position_x() const { return x[0]; }
float OurEKF::get_position_y() const { return x[1]; }
float OurEKF::get_position_z() const { return x[2]; }

float OurEKF::get_velocity_x() const { return x[3]; }
float OurEKF::get_velocity_y() const { return x[4]; }
float OurEKF::get_velocity_z() const { return x[5]; }

float OurEKF::get_accel_bias_x() const { return x[10]; }
float OurEKF::get_accel_bias_y() const { return x[11]; }
float OurEKF::get_accel_bias_z() const { return x[12]; }

float OurEKF::get_gyro_bias_x() const { return x[13]; }
float OurEKF::get_gyro_bias_y() const { return x[14]; }
float OurEKF::get_gyro_bias_z() const { return x[15]; }

float OurEKF::get_roll() const  { return roll; }
float OurEKF::get_pitch() const { return pitch; }
float OurEKF::get_yaw() const   { return yaw; }

void OurEKF::get_state(float state[STATE_SIZE]) const
{
    for (int i = 0; i < STATE_SIZE; i++) {
        state[i] = x[i];
    }
}

void OurEKF::get_covariance(float covariance[STATE_SIZE][STATE_SIZE]) const
{
    for (int i = 0; i < STATE_SIZE; i++) {
        for (int j = 0; j < STATE_SIZE; j++) {
            covariance[i][j] = P[i][j];
        }
    }
}

float OurEKF::get_innovation_x() const { return innovation[0]; }
float OurEKF::get_innovation_y() const { return innovation[1]; }
float OurEKF::get_innovation_z() const { return innovation[2]; }

Vector3f OurEKF::get_velocity_innovation() const
{
    return Vector3f(innovation_vel[0], innovation_vel[1], innovation_vel[2]);
}

Vector3f OurEKF::get_mag_innovation() const
{
    return Vector3f(innovation_mag[0], innovation_mag[1], innovation_mag[2]);
}

bool OurEKF::is_initialized() const
{
    return initialized;
}


// ============================================================================
// MATRIX HELPERS
// ============================================================================

void OurEKF::zero_matrix(float A[STATE_SIZE][STATE_SIZE])
{
    memset(A, 0, sizeof(float) * STATE_SIZE * STATE_SIZE);
}


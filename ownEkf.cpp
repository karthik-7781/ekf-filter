#include "ownEkf.h"

#include <string.h>
#include <math.h>
#include <AP_GPS/AP_GPS.h>

#include <AP_HAL/AP_HAL.h>
extern const AP_HAL::HAL& hal;
//#include <AP_NavEKF/AP_NavEKF.h>


// Constructor


OurEKF::OurEKF()
{
    memset(x, 0, sizeof(x));
    memset(P, 0, sizeof(P));
    memset(F, 0, sizeof(F));
    memset(Q, 0, sizeof(Q));
    memset(H, 0, sizeof(H));
    memset(R, 0, sizeof(R));
    memset(S, 0, sizeof(S));
    memset(K, 0, sizeof(K));
    memset(innovation, 0, sizeof(innovation));

    roll=0.0f;
    pitch=0.0f;
    yaw=35.11f;

    initialized = false;

    // calibration defaults
    calib_active = false;
    calib_target = 0;
    calib_count = 0;
    calib_sum_measured = Vector3f(0.0f, 0.0f, 0.0f);
    calib_sum_expected = Vector3f(0.0f, 0.0f, 0.0f);
}


// Initialization


void OurEKF::init(const Vector3f &position,
                  const Vector3f &velocity,
                  const Quaternion &quaternion)
{
    memset(x, 0, sizeof(x));
    memset(P, 0, sizeof(P));

    // Initial state


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

    // Initial accelerometer bias
    x[10] = 0.0f;
    x[11] = 0.0f;
    x[12] = 0.0f;

    // Initial gyro bias
    x[13] = 0.0f;
    x[14] = 0.0f;
    x[15] = 0.0f;

   
    // Normalize quaternion
  

    normalize_quaternion();

  
    // Initial covariance
  
    // These are example values.
    // They MUST eventually be tuned for the actual system.
    
    

    for (int i = 0; i < STATE_SIZE; i++) {
        P[i][i] = 0.01f;
    }

    // Position uncertainty
    P[0][0] = 1.0f;
    P[1][1] = 1.0f;
    P[2][2] = 1.0f;

    // Velocity uncertainty
    P[3][3] = 0.5f;
    P[4][4] = 0.5f;
    P[5][5] = 0.5f;

    // Quaternion uncertainty
    P[6][6] = 0.01f;
    P[7][7] = 0.01f;
    P[8][8] = 0.01f;
    P[9][9] = 0.01f;

    // Accelerometer bias uncertainty
    P[10][10] = 0.01f;
    P[11][11] = 0.01f;
    P[12][12] = 0.01f;

    // Gyro bias uncertainty
    P[13][13] = 0.01f;
    P[14][14] = 0.01f;
    P[15][15] = 0.01f;

    initialized = true;
}



// Complete EKF update


void OurEKF::update(const Vector3f &accel,
                    const Vector3f &gyro,
                    const Vector3f &mag,
                    float dt,
                    bool gps_available,
                    const Vector3f &gps_position)
{
    if (!initialized) {
        return;
    }  

    // IMU Prediction
    predict(accel, gyro, dt); 

    // GPS Correction
    if (gps_available) {
        update_gps(gps_position);
    }

    // Magnetometer Correction
    //update_mag(mag);
}
// Prediction
void OurEKF::predict(const Vector3f &accel,
                     const Vector3f &gyro,
                     float dt)
{
    if (!initialized) {
        return;
    }

    if (dt <= 0.0f) {
        return;
    }


    // STAGE 2
    // Remove estimated sensor biases
 

    Vector3f accel_corrected;
    Vector3f gyro_corrected;

    accel_corrected.x = accel.x - x[10];
    accel_corrected.y = accel.y - x[11];
    accel_corrected.z = accel.z - x[12];

    gyro_corrected.x = gyro.x - x[13];
    gyro_corrected.y = gyro.y - x[14];
    gyro_corrected.z = gyro.z - x[15];

    last_accel_body = accel_corrected;
    
    // STAGE 3
    // Quaternion prediction
   

    propagate_quaternion(gyro_corrected, dt);


  
    // STAGE 4
    // Body frame -> world frame
    

    Quaternion q = get_state_quaternion();

    Matrix3f R_body_to_world =
        quaternion_to_rotation_matrix(q);

    Vector3f accel_world;

    accel_world.x =
        R_body_to_world.a.x * accel_corrected.x +
        R_body_to_world.b.x * accel_corrected.y +
        R_body_to_world.c.x * accel_corrected.z;

    accel_world.y =
        R_body_to_world.a.y * accel_corrected.x +
        R_body_to_world.b.y * accel_corrected.y +
        R_body_to_world.c.y * accel_corrected.z;

    accel_world.z =
        R_body_to_world.a.z * accel_corrected.x +
        R_body_to_world.b.z * accel_corrected.y +
        R_body_to_world.c.z * accel_corrected.z;


    // STAGE 5
    // Gravity compensation
    
    // Current convention:
    // world +Z = up
    // gravity = [0,0,-9.81]
    
    // Specific-force convention:
    
    // linear acceleration =
    //       rotated acceleration + [0,0,9.81]
    

    accel_world.z += 9.81f;

    last_accel_world = accel_world; 


    
    // STAGE 7
    // Position prediction
    
    // p = p + v*dt + 0.5*a*dt^2
   

    float half_dt2 = 0.5f * dt * dt;

    x[0] += x[3] * dt + accel_world.x * half_dt2;
    x[1] += x[4] * dt + accel_world.y * half_dt2;
    x[2] += x[5] * dt + accel_world.z * half_dt2;


    
    // STAGE 6
    // Velocity prediction
    
    // v = v + a*dt
    x[3] += accel_world.x * dt;
    x[4] += accel_world.y * dt;
    x[5] += accel_world.z * dt;


    // STAGE 8
    // Bias prediction
    
    // Biases are random walk states.
    
    // b(k) = b(k-1) + w
    
    // Mean prediction therefore keeps the current value.
    


    // STAGE 9
    // Build F
   

    build_F(accel_corrected, dt);


    // STAGE 10
    // Build Q
    

    build_Q(dt);



    // STAGE 11
    // Covariance prediction
    
    // P = F P F^T + Q
  

    predict_covariance();


   
    // Quaternion normalization
   

    normalize_quaternion();

    Orientation();

    hal.console->printf("GYRO: %.4f %.4f %.4f\n",
    gyro.x, gyro.y, gyro.z);

    hal.console->printf("ACCEL: %.4f %.4f %.4f\n",
    accel.x, accel.y, accel.z);

    hal.console->printf("DT: %.6f\n", dt);

    hal.console->printf("Q: %.4f %.4f %.4f %.4f\n",
    x[6], x[7], x[8], x[9]);

    hal.console->printf("RPY: %.2f %.2f %.2f\n",
    get_roll()  * RAD_TO_DEG,
    get_pitch() * RAD_TO_DEG,
    get_yaw()   * RAD_TO_DEG);


    hal.console->printf("GPS K quat:\n");

    hal.console->printf("Kq0: %.6f %.6f %.6f\n",
    K[6][0], K[6][1], K[6][2]);

    hal.console->printf("Kq1: %.6f %.6f %.6f\n",
    K[7][0], K[7][1], K[7][2]);

    hal.console->printf("Kq2: %.6f %.6f %.6f\n",
    K[8][0], K[8][1], K[8][2]);

    hal.console->printf("Kq3: %.6f %.6f %.6f\n",
       K[9][0], K[9][1], K[9][2]);
}



// Quaternion prediction


void OurEKF::propagate_quaternion(const Vector3f &gyro,
                                  float dt)
{
    Quaternion q = get_state_quaternion();

    Quaternion dq = quaternion_from_gyro(gyro, dt);

    // Quaternion multiplication
    Quaternion q_new;

    q_new.q1 =
        q.q1 * dq.q1 -
        q.q2 * dq.q2 -
        q.q3 * dq.q3 -
        q.q4 * dq.q4;

    q_new.q2 =
        q.q1 * dq.q2 +
        q.q2 * dq.q1 +
        q.q3 * dq.q4 -
        q.q4 * dq.q3;

    q_new.q3 =
        q.q1 * dq.q3 -
        q.q2 * dq.q4 +
        q.q3 * dq.q1 +
        q.q4 * dq.q2;

    q_new.q4 =
        q.q1 * dq.q4 +
        q.q2 * dq.q3 -
        q.q3 * dq.q2 +
        q.q4 * dq.q1;

    set_state_quaternion(q_new);

    normalize_quaternion();
}

void OurEKF::Orientation()
{
    Quaternion q = get_state_quaternion();

    float qw = q.q1;
    float qx = q.q2;
    float qy = q.q3;
    float qz = q.q4;

    // Roll
    roll = atan2f(
        2.0f * (qw * qx + qy * qz),
        1.0f - 2.0f * (qx * qx + qy * qy)
    );

    // Pitch
    float sin_pitch =
        2.0f * (qw * qy - qz * qx);

    // protect against floating point values slightly > 1 or < -1
    if (sin_pitch > 1.0f) {
        sin_pitch = 1.0f;
    }

    if (sin_pitch < -1.0f) {
        sin_pitch = -1.0f;
    }

    pitch = asinf(sin_pitch);

    // Yaw
    yaw = atan2f(
        2.0f * (qw * qz + qx * qy),
        1.0f - 2.0f * (qy * qy + qz * qz)
    );
}



// Create delta quaternion from gyro


Quaternion OurEKF::quaternion_from_gyro(const Vector3f &gyro,
                                         float dt) const
{
    float angle_x = gyro.x * dt;
    float angle_y = gyro.y * dt;
    float angle_z = gyro.z * dt;

    float angle =
        sqrtf(angle_x * angle_x +
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

    float half_angle = 0.5f * angle;

    float s = sinf(half_angle) / angle;

    dq.q1 = cosf(half_angle);
    dq.q2 = angle_x * s;
    dq.q3 = angle_y * s;
    dq.q4 = angle_z * s;

    return dq;
}


// Quaternion -> rotation matrix


Matrix3f OurEKF::quaternion_to_rotation_matrix(
    const Quaternion &q) const
{
    Matrix3f Root;

    float qw = q.q1;
    float qx = q.q2;
    float qy = q.q3;
    float qz = q.q4;

    Root.a.x = 1.0f - 2.0f * (qy * qy + qz * qz);
    Root.a.y = 2.0f * (qx * qy + qz * qw);
    Root.a.z = 2.0f * (qx * qz - qy * qw);

    Root.b.x = 2.0f * (qx * qy - qz * qw);
    Root.b.y = 1.0f - 2.0f * (qx * qx + qz * qz);
    Root.b.z = 2.0f * (qy * qz + qx * qw);

    Root.c.x = 2.0f * (qx * qz + qy * qw);
    Root.c.y = 2.0f * (qy * qz - qx * qw);
    Root.c.z = 1.0f - 2.0f * (qx * qx + qy * qy);

    return Root;
}


// Build F


void OurEKF::build_F(const Vector3f &accel_body,
                     float dt)
{
    zero_matrix_16(F);

    // Identity blocks
    

    F[0][0] = 1.0f;
    F[1][1] = 1.0f;
    F[2][2] = 1.0f;

    F[3][3] = 1.0f;
    F[4][4] = 1.0f;
    F[5][5] = 1.0f;

    F[6][6] = 1.0f;
    F[7][7] = 1.0f;
    F[8][8] = 1.0f;
    F[9][9] = 1.0f;

    F[10][10] = 1.0f;
    F[11][11] = 1.0f;
    F[12][12] = 1.0f;

    F[13][13] = 1.0f;
    F[14][14] = 1.0f;
    F[15][15] = 1.0f;

    // Position <- velocity
    
    // p(k+1) = p(k) + v(k)dt + ...
   

    F[0][3] = dt;
    F[1][4] = dt;
    F[2][5] = dt;


    
    // Position <- accelerometer bias
    
    // approximately:
    
    // F_p,ba = -0.5 R dt²


    Quaternion q = get_state_quaternion();

    Matrix3f Rot = quaternion_to_rotation_matrix(q);


    float half_dt2 = 0.5f * dt * dt;

    F[0][10] = -half_dt2 * Rot.a.x;
    F[0][11] = -half_dt2 * Rot.b.x;
    F[0][12] = -half_dt2 * Rot.c.x;

    F[1][10] = -half_dt2 * Rot.a.y;
    F[1][11] = -half_dt2 * Rot.b.y;
    F[1][12] = -half_dt2 * Rot.c.y;

    F[2][10] = -half_dt2 * Rot.a.z;
    F[2][11] = -half_dt2 * Rot.b.z;
    F[2][12] = -half_dt2 * Rot.c.z;


    // Velocity <- accelerometer bias
   
    // F_v,ba = -R dt
  

    F[3][10] = -dt * Rot.a.x;
    F[3][11] = -dt * Rot.b.x;
    F[3][12] = -dt * Rot.c.x;

    F[4][10] = -dt * Rot.a.y;
    F[4][11] = -dt * Rot.b.y;
    F[4][12] = -dt * Rot.c.y;

    F[5][10] = -dt * Rot.a.z;
    F[5][11] = -dt * Rot.b.z;
    F[5][12] = -dt * Rot.c.z;


    // Quaternion / gyro-bias relationships
    
    // A first-order approximation is used here.
    
    // q(k+1) ≈ q(k) + 0.5 Ω(q) ω dt
    
    // Therefore:
    
    // F_q,q ≈ I + 0.5 Ω(ω)dt
    
    // F_q,bg ≈ -0.5 Ω(q)dt


    Vector3f gyro;

    // We do not have the raw gyro inside this function.
    // Use zero angular rate for the linearized quaternion block.
    gyro.x = 0.0f;
    gyro.y = 0.0f;
    gyro.z = 0.0f;

    float half_dt = 0.5f * dt;

    F[6][6] = 1.0f;

    F[7][7] = 1.0f;
    F[8][8] = 1.0f;
    F[9][9] = 1.0f;

  
    // qdot = 0.5 * q ⊗ [0,omega]
   

    float qw = q.q1;
    float qx = q.q2;
    float qy = q.q3;
    float qz = q.q4;

    F[6][13] =  half_dt * qx;
    F[6][14] =  half_dt * qy;
    F[6][15] =  half_dt * qz;

    F[7][13] = -half_dt * qw;
    F[7][14] =  half_dt * qz;
    F[7][15] = -half_dt * qy;

    F[8][13] = -half_dt * qz;
    F[8][14] = -half_dt * qw;
    F[8][15] =  half_dt * qx;

    F[9][13] =  half_dt * qy;
    F[9][14] = -half_dt * qx;
    F[9][15] = -half_dt * qw;


    
    // Approximate attitude -> acceleration coupling.
    
    // For the initial implementation this is intentionally simplified.
    // A full analytical J_q should be derived consistently with the exact
    // quaternion convention before using this EKF for serious navigation.
   

    (void)accel_body;
}



// Build Q


void OurEKF::build_Q(float dt)
{
    zero_matrix_16(Q);


    // Example sensor noise parameters.
    
    // These are NOT universal values.
    // They must be obtained/tuned from the actual IMU.
    

    const float accel_noise_std = 0.20f;
    const float gyro_noise_std  = 0.02f;

    const float accel_bias_rw_std = 0.001f;
    const float gyro_bias_rw_std  = 0.0005f;

    const float accel_var =
        accel_noise_std * accel_noise_std;

    const float gyro_var =
        gyro_noise_std * gyro_noise_std;

    const float accel_bias_var =
        accel_bias_rw_std * accel_bias_rw_std;

    const float gyro_bias_var =
        gyro_bias_rw_std * gyro_bias_rw_std;


    
    // Position process noise
    
    // Q_p ≈ 1/4 R Sigma_a R^T dt^4
    
    // Simplified here as diagonal.
    
    float dt2 = dt * dt;
    float dt4 = dt2 * dt2;

    float position_noise =
        0.25f * accel_var * dt4;

    Q[0][0] = position_noise;
    Q[1][1] = position_noise;
    Q[2][2] = position_noise;


    // Velocity process noise
    
    // Q_v ≈ R Sigma_a R^T dt²
    
    // Simplified diagonal.
   

    float velocity_noise =
        accel_var * dt2;

    Q[3][3] = velocity_noise;
    Q[4][4] = velocity_noise;
    Q[5][5] = velocity_noise;


   
    // Quaternion process noise
  
    float quaternion_noise =
        0.25f * gyro_var * dt2;

    Q[6][6] = quaternion_noise;
    Q[7][7] = quaternion_noise;
    Q[8][8] = quaternion_noise;
    Q[9][9] = quaternion_noise;


   
    // Accelerometer bias random walk
  
    Q[10][10] = accel_bias_var * dt;
    Q[11][11] = accel_bias_var * dt;
    Q[12][12] = accel_bias_var * dt;


    
    // Gyro bias random walk
    
    Q[13][13] = gyro_bias_var * dt;
    Q[14][14] = gyro_bias_var * dt;
    Q[15][15] = gyro_bias_var * dt;
}



// Covariance prediction

// P = F P F^T + Q


void OurEKF::predict_covariance()
{
    float temp[16][16] = {};

    
    // temp = F * P
    
    for (int i = 0; i < 16; i++) {
        for (int j = 0; j < 16; j++) {

            float sum = 0.0f;

            for (int k = 0; k < 16; k++) {
                sum += F[i][k] * P[k][j];
            }

            temp[i][j] = sum;
        }
    }


    // P_new = temp * F^T + Q
   
    for (int i = 0; i < 16; i++) {
        for (int j = 0; j < 16; j++) {

            float sum = 0.0f;

            for (int k = 0; k < 16; k++) {
                sum += temp[i][k] * F[j][k];
            }

            P[i][j] = sum + Q[i][j];
        }
    }
}


// GPS UPDATE


bool OurEKF::update_gps(const Vector3f &gps_position)
{
    int i=0, j=0;
    if (!initialized) {
        return false;
    }


  
    // STAGE 13 / 14
    // Measurement model and H


    build_H();



    // STAGE 15
    // Predicted measurement
    

    float z_pred[3];

    z_pred[0] = x[0];
    z_pred[1] = x[1];
    z_pred[2] = x[2];


    
    // STAGE 16
    // Innovation
    
    // y = z - z_hat
    

    innovation[0] =
        gps_position.x - z_pred[0];

    innovation[1] =
        gps_position.y - z_pred[1];

    innovation[2] =
        gps_position.z - z_pred[2];


   
    // STAGE 17
    // Measurement covariance R
    
    build_R();


    
    // STAGE 18
    // Innovation covariance S
    
    // S = H P H^T + R
   

    if (!calculate_innovation_covariance()) {
        return false;
    }


   
    // STAGE 19
    // Kalman gain
    
    // K = P H^T S^-1
    
    if (!calculate_kalman_gain()) {
        return false;
    }


   
    // STAGE 20
    // State correction
    
    // x = x + K y
    

    correct_state();


    
    // STAGE 21
    // Covariance correction
   

    correct_covariance();


    
    // STAGE 22
    // Quaternion normalization
    

    normalize_quaternion();

    x[i] += K[i][j] * innovation[j];

    return true;
}




// bool OurEKF::update_mag(const Vector3f &mag)
// {
//     if (!initialized) {
//         return false;
//     }

    
//     // 1. Calculate magnetic heading from existing AP_Compass data
   
//     float mag_yaw = atan2f(-mag.y, mag.x);

   
//     // 2. Get current EKF yaw from quaternion
    
//     Quaternion q = get_state_quaternion();

//     float ekf_yaw = atan2f(
//         2.0f * (q.q1 * q.q4 + q.q2 * q.q3),
//         1.0f - 2.0f * (q.q3 * q.q3 + q.q4 * q.q4)
//     );

   
//     // 3. Calculate yaw innovation
   

//     float yaw_error = mag_yaw - ekf_yaw;

//     // Wrap to -PI ... +PI
//     while (yaw_error > M_PI) {
//         yaw_error -= 2.0f * M_PI;
//     }

//     while (yaw_error < -M_PI) {
//         yaw_error += 2.0f * M_PI;
//     }

//     innovation_mag[0] = yaw_error;

   
//     // 4. Measurement Jacobian
    
//     // For this simple implementation we treat yaw as the
//     // measurement and use a simple sensitivity to the
//     // quaternion attitude states.
    

//     // for (int i = 0; i < STATE_SIZE; i++) {
//     //     H_mag[0][i] = 0.0f;
//     // }

//     // // Numerical/simple yaw sensitivity.
  
//     // // This is intentionally simple for your first implementation.
//     // H_mag[0][8] = 1.0f;
//     // H_mag[0][9] = 1.0f;



//     float qw = q.q1;
//     float qx = q.q2;
//     float qy = q.q3;
//     float qz = q.q4;

//     float a = 2.0f * (qw*qz + qx*qy);
//     float b = 1.0f - 2.0f * (qy*qy + qz*qz);

//     float denom = a*a + b*b;
    
//     for (int i = 0; i < STATE_SIZE; i++) {
//     H_mag[0][i] = 0.0f;
//     }


//     H_mag[0][6] = (b * (2.0f*qz)) / denom;                       // d yaw / d qw

//     H_mag[0][7] = (b * (2.0f*qy)) / denom;                       // d yaw / d qx

//     H_mag[0][8] = (b * (2.0f*qx) - a * (-4.0f*qy)) / denom;     // d yaw / d qy

//     H_mag[0][9] = (b * (2.0f*qw) - a * (-4.0f*qz)) / denom;     // d yaw / d qz

//     // 5. Measurement noise
   

//     R_mag[0][0] = sq(5.0f * DEG_TO_RAD);

   
//     // 6. Innovation covariance

//     // S = HPH' + R


//     float S_value = R_mag[0][0];

//     for (int i = 0; i < STATE_SIZE; i++) {
//         for (int j = 0; j < STATE_SIZE; j++) {
//             S_value += H_mag[0][i] *
//                        P[i][j] *
//                        H_mag[0][j];
//         }
//     }

//     if (S_value <= 0.0f) {
//         return false;
//     }

//     S_mag[0][0] = S_value;

   
//     // 7. Kalman gain
   
//     // K = P H' / S
   
//     for (int i = 0; i < STATE_SIZE; i++) {

//         float value = 0.0f;

//         for (int j = 0; j < STATE_SIZE; j++) {
//             value += P[i][j] * H_mag[0][j];
//         }

//         K_mag[i][0] = value / S_value;
//     }

   
//     // 8. Correct state
  
//     // x = x + K * innovation
   
//     for (int i = 0; i < STATE_SIZE; i++) {
//         x[i] += K_mag[i][0] * innovation_mag[0];
//     }

  
//     // 9. Normalize quaternion
    

//     normalize_quaternion();

    
//     // 10. Correct covariance
    
//     // P = (I - KH)P
   

//     float P_new[STATE_SIZE][STATE_SIZE];

//     for (int i = 0; i < STATE_SIZE; i++) {

//         for (int j = 0; j < STATE_SIZE; j++) {

//             float value = P[i][j];

//             for (int k = 0; k < STATE_SIZE; k++) {
//                 value -= K_mag[i][0] *
//                          H_mag[0][k] *
//                          P[k][j];
//             }

//             P_new[i][j] = value;
//         }
//     }

//     for (int i = 0; i < STATE_SIZE; i++) {
//         for (int j = 0; j < STATE_SIZE; j++) {
//             P[i][j] = P_new[i][j];
//         }
//     }

//     return true;
// }

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
//     // ---------------------------------------------------------

//     float a = 2.0f * (qw*qz + qx*qy);

//     float b = 1.0f - 2.0f * (qy*qy + qz*qz);

//     float ekf_yaw = atan2f(a, b);

    
//     // 4. Innovation
    

//     float yaw_error = mag_yaw - ekf_yaw;

//     while (yaw_error > M_PI) {
//         yaw_error -= 2.0f * M_PI;
//     }

//     while (yaw_error < -M_PI) {
//         yaw_error += 2.0f * M_PI;
//     }

//     innovation_mag[0] = yaw_error;

//     // ---------------------------------------------------------
//     // 5. Build H
//     // ---------------------------------------------------------

//     for (int i = 0; i < STATE_SIZE; i++) {
//         H_mag[0][i] = 0.0f;
//     }

//     float denom = a*a + b*b;

//     if (denom < 1.0e-6f) {
//         return false;
//     }


//     // dN/dq
//     float dN_dqw = 2.0f * qz;
//     float dN_dqx = 2.0f * qy;
//     float dN_dqy = 2.0f * qx;
//     float dN_dqz = 2.0f * qw;

//     // dD/dq
//     float dD_dqw = 0.0f;
//     float dD_dqx = 0.0f;
//     float dD_dqy = -4.0f * qy;
//     float dD_dqz = -4.0f * qz;




//     // d(yaw)/d(qw)
//     H_mag[0][6] =
//         (b * (2.0f*qz)) / denom;

//     // d(yaw)/d(qx)
//     H_mag[0][7] =
//         (b * (2.0f*qy)) / denom;

//     // d(yaw)/d(qy)
//     H_mag[0][8] =
//         (b * (2.0f*qx) -
//          a * (-4.0f*qy)) / denom;

//     // d(yaw)/d(qz)
//     H_mag[0][9] =
//         (b * (2.0f*qw) -
//          a * (-4.0f*qz)) / denom;

//     // ---------------------------------------------------------
//     // 6. Measurement noise
//     // ---------------------------------------------------------

//     R_mag[0][0] = sq(5.0f * DEG_TO_RAD);

//     // ---------------------------------------------------------
//     // 7. S = HPH' + R
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

//     // ---------------------------------------------------------
//     // 8. K = PH'/S
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
//     // 9. State correction
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
//     // 11. Covariance correction
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

//     for (int i = 0; i < STATE_SIZE; i++) {
//         for (int j = 0; j < STATE_SIZE; j++) {
//             P[i][j] = P_new[i][j];
//         }
//     }

//     return true;
// }


bool OurEKF::update_mag(const Vector3f &mag)
{
    if (!initialized) {
        return false;
    }

    // ---------------------------------------------------------
    // 1. Calculate magnetic heading
    // ---------------------------------------------------------

    float mag_yaw = atan2f(-mag.y, mag.x);

    // ---------------------------------------------------------
    // 2. Get current EKF quaternion
    // ---------------------------------------------------------

    Quaternion q = get_state_quaternion();

    float qw = q.q1;
    float qx = q.q2;
    float qy = q.q3;
    float qz = q.q4;

    // ---------------------------------------------------------
    // 3. Calculate current EKF yaw
    //
    // yaw = atan2(a, b)
    //
    // a = 2(qw*qz + qx*qy)
    // b = 1 - 2(qy^2 + qz^2)
    // ---------------------------------------------------------

    float a = 2.0f * (qw*qz + qx*qy);

    float b = 1.0f - 2.0f * (qy*qy + qz*qz);

    float ekf_yaw = atan2f(a, b);

    // ---------------------------------------------------------
    // 4. Calculate yaw innovation
    // ---------------------------------------------------------

    float yaw_error = mag_yaw - ekf_yaw;

    // Wrap innovation to [-PI, +PI]
    while (yaw_error > M_PI) {
        yaw_error -= 2.0f * M_PI;
    }

    while (yaw_error < -M_PI) {
        yaw_error += 2.0f * M_PI;
    }

    innovation_mag[0] = yaw_error;

    // ---------------------------------------------------------
    // 5. Build measurement Jacobian H
    //
    // Measurement:
    //
    // z = yaw
    //
    // H = d(yaw)/d(state)
    //
    // Only quaternion states are involved:
    //
    // x[6] = qw
    // x[7] = qx
    // x[8] = qy
    // x[9] = qz
    // ---------------------------------------------------------

    for (int i = 0; i < STATE_SIZE; i++) {
        H_mag[0][i] = 0.0f;
    }

    float denom = a*a + b*b;

    if (denom < 1.0e-6f) {
        return false;
    }

    // ---------------------------------------------------------
    // Derivatives of a
    //
    // a = 2(qw*qz + qx*qy)
    // ---------------------------------------------------------

    float da_dqw = 2.0f * qz;
    float da_dqx = 2.0f * qy;
    float da_dqy = 2.0f * qx;
    float da_dqz = 2.0f * qw;

    // ---------------------------------------------------------
    // Derivatives of b
    //
    // b = 1 - 2(qy^2 + qz^2)
    // ---------------------------------------------------------

    float db_dqw = 0.0f;
    float db_dqx = 0.0f;
    float db_dqy = -4.0f * qy;
    float db_dqz = -4.0f * qz;

    // ---------------------------------------------------------
    // d atan2(a,b) / dq
    //
    // dyaw/dq =
    //
    // (b * da/dq - a * db/dq)
    // ------------------------
    //       a^2 + b^2
    // ---------------------------------------------------------

    H_mag[0][6] =
        (b * da_dqw - a * db_dqw) / denom;

    H_mag[0][7] =
        (b * da_dqx - a * db_dqx) / denom;

    H_mag[0][8] =
        (b * da_dqy - a * db_dqy) / denom;

    H_mag[0][9] =
        (b * da_dqz - a * db_dqz) / denom;

    // ---------------------------------------------------------
    // 6. Magnetometer measurement noise
    //
    // 5 degrees standard deviation
    // ---------------------------------------------------------

    R_mag[0][0] =
        sq(5.0f * DEG_TO_RAD);

    // ---------------------------------------------------------
    // 7. Innovation covariance
    //
    // S = H P H' + R
    // ---------------------------------------------------------

    float S_value = R_mag[0][0];

    for (int i = 0; i < STATE_SIZE; i++) {

        for (int j = 0; j < STATE_SIZE; j++) {

            S_value +=
                H_mag[0][i] *
                P[i][j] *
                H_mag[0][j];
        }
    }

    if (S_value <= 1.0e-9f) {
        return false;
    }

    S_mag[0][0] = S_value;

    // ---------------------------------------------------------
    // 8. Kalman gain
    //
    // K = P H' / S
    // ---------------------------------------------------------

    for (int i = 0; i < STATE_SIZE; i++) {

        float value = 0.0f;

        for (int j = 0; j < STATE_SIZE; j++) {

            value +=
                P[i][j] *
                H_mag[0][j];
        }

        K_mag[i][0] = value / S_value;
    }

    // ---------------------------------------------------------
    // 9. Correct state
    //
    // x = x + K * innovation
    // ---------------------------------------------------------

    for (int i = 0; i < STATE_SIZE; i++) {

        x[i] +=
            K_mag[i][0] *
            innovation_mag[0];
    }

    // ---------------------------------------------------------
    // 10. Normalize quaternion
    // ---------------------------------------------------------

    normalize_quaternion();

    // ---------------------------------------------------------
    // 11. Correct covariance
    //
    // P = (I - K H) P
    // ---------------------------------------------------------

    float P_new[STATE_SIZE][STATE_SIZE];

    for (int i = 0; i < STATE_SIZE; i++) {

        for (int j = 0; j < STATE_SIZE; j++) {

            float value = P[i][j];

            for (int k = 0; k < STATE_SIZE; k++) {

                value -=
                    K_mag[i][0] *
                    H_mag[0][k] *
                    P[k][j];
            }

            P_new[i][j] = value;
        }
    }

    // Copy corrected covariance
    for (int i = 0; i < STATE_SIZE; i++) {

        for (int j = 0; j < STATE_SIZE; j++) {

            P[i][j] = P_new[i][j];
        }
    }

    hal.console->printf(
    "MAG yaw=%.2f EKF yaw=%.2f error=%.2f\n",
    mag_yaw * RAD_TO_DEG,
    ekf_yaw * RAD_TO_DEG,
    yaw_error * RAD_TO_DEG
);

    return true;
}


// H

// GPS directly measures:

// z = [px py pz]

// Therefore:

// H =
// [1 0 0 ...]
// [0 1 0 ...]
// [0 0 1 ...]

//Measurement Model tells which state varible influences this measurement

//after kalman gain, the row related to bax will have non-zero, so bias can be updated.



void OurEKF::build_H()
{
    zero_matrix_3x16(H);

    H[0][0] = 1.0f;
    H[1][1] = 1.0f;
    H[2][2] = 1.0f;
}



// R measurement covariance or uncertainity 

void OurEKF::build_R()
{
    zero_matrix_3(R);

    // Example GPS standard deviations.
    // Units: metres.

    const float gps_x_std = 2.0f;
    const float gps_y_std = 2.0f;
    const float gps_z_std = 4.0f;

    R[0][0] =
        gps_x_std * gps_x_std;

    R[1][1] =
        gps_y_std * gps_y_std;

    R[2][2] =
        gps_z_std * gps_z_std;
}



// Calculate S

// S = H P H^T + R

// Since H selects position:

// S =
// [P00 P01 P02]
// [P10 P11 P12]
// [P20 P21 P22]
//
// + R



// S = HPH^T + R

bool OurEKF::calculate_innovation_covariance()
{
    zero_matrix_3(S);

    for (int i = 0; i < GPS_SIZE; i++) {

        for (int j = 0; j < GPS_SIZE; j++) {

            for (int k = 0; k < STATE_SIZE; k++) {

                for (int l = 0; l < STATE_SIZE; l++) {

                    S[i][j] += H[i][k] * P[k][l] * H[j][l];
                }
            }

          S[i][j] += R[i][j];  
        }
        
    }

    return true;
}



// Calculate Kalman gain

// K = P*H^T*S^-1

//decides whom to believe more, the prediction or the measurement.

bool OurEKF::calculate_kalman_gain()
{
    float S_inv[3][3] = {};

    if (!inverse_3x3(S, S_inv)) {
        return false;
    }


   
    // PH^T
    
    float PHt[STATE_SIZE][GPS_SIZE] = {};

    for (int i = 0; i < STATE_SIZE; i++) {

        for (int j = 0; j < GPS_SIZE; j++) {

            for (int k = 0; k < STATE_SIZE; k++) {

                PHt[i][j] +=
                    P[i][k] *
                    H[j][k];
            }
        }
    }


   
    // K = PH^T S^-1
  
    for (int i = 0; i < STATE_SIZE; i++) {

        for (int j = 0; j < GPS_SIZE; j++) {

            K[i][j] = 0.0f;

            for (int k = 0; k < GPS_SIZE; k++) {

                K[i][j] +=
                    PHt[i][k] *
                    S_inv[k][j];
            }
        }
    }

    return true;
}



// Correct state

// x = x + K innovation

// x = K + Innovation

void OurEKF::correct_state()
{
    float dx[STATE_SIZE] = {};

    for (int i = 0; i < STATE_SIZE; i++) {

        for (int j = 0; j < GPS_SIZE; j++) {

            dx[i] +=
                K[i][j] *
                innovation[j];
        }
    }


    for (int i = 0; i < STATE_SIZE; i++) {

        x[i] += dx[i];
    }
}



// Correct covariance

// Joseph form:

// P = (I-KH) P (I-KH)^T + K R K^T

// This is numerically safer than simply:

// P = (I-KH)P


void OurEKF::correct_covariance()
{
   

    
    // temp = (I - K*H)
   
    for (int i = 0; i < 16; i++) {
        for (int j = 0; j < 16; j++) {

            float kh = 0.0f;

            for (int k = 0; k < 3; k++) {
                kh += K[i][k] * H[k][j];
            }

            cov_temp1[i][j] = (i == j ? 1.0f : 0.0f) - kh;
        }
    }

   
    // P_new = (I-KH) * P * (I-KH)^T
    
    // We calculate one row/column at a time and store the
    // result back into P only after the required old P values
    // have been used.
    

    for (int i = 0; i < 16; i++) {
        for (int j = 0; j < 16; j++) {
            cov_temp2[i][j] = P[i][j];
        }
    }

    for (int i = 0; i < 16; i++) {
        for (int j = 0; j < 16; j++) {

            float value = 0.0f;

            for (int k = 0; k < 16; k++) {
                for (int l = 0; l < 16; l++) {
                    value += cov_temp1[i][k] *
                             cov_temp2[k][l] *
                             cov_temp1[j][l];
                }
            }

            // K * R * K^T
            float krkt = 0.0f;

            for (int k = 0; k < 3; k++) {
                for (int l = 0; l < 3; l++) {
                    krkt += K[i][k] *
                            R[k][l] *
                            K[j][l];
                }
            }

            P[i][j] = value + krkt;
        }
    }
}





// Quaternion normalization

void OurEKF::normalize_quaternion()
{
    float qw = x[6];
    float qx = x[7];
    float qy = x[8];
    float qz = x[9];

    float norm =
        sqrtf(
            qw * qw +
            qx * qx +
            qy * qy +
            qz * qz
        );

    if (norm > 1.0e-6f) {

        x[6] /= norm;
        x[7] /= norm;
        x[8] /= norm;
        x[9] /= norm;
    }
    else {

        // Safe fallback
        x[6] = 1.0f;
        x[7] = 0.0f;
        x[8] = 0.0f;
        x[9] = 0.0f;
    }
}



// Get quaternion


Quaternion OurEKF::get_state_quaternion() const
{
    Quaternion q;

    q.q1 = x[6];
    q.q2 = x[7];
    q.q3 = x[8];
    q.q4 = x[9];

    return q;
}

// Set quaternion

void OurEKF::set_state_quaternion(const Quaternion &q)
{
    x[6] = q.q1;
    x[7] = q.q2;
    x[8] = q.q3;
    x[9] = q.q4;
}


// Position getter

Vector3f OurEKF::get_position() const
{
    return Vector3f(
        x[0],
        x[1],
        x[2]
    );
}



// Velocity getter


Vector3f OurEKF::get_velocity() const
{
    return Vector3f(
        x[3],
        x[4],
        x[5]
    );
}



// Quaternion getter


Quaternion OurEKF::get_quaternion() const
{
    return get_state_quaternion();
}


// Accelerometer bias getter


Vector3f OurEKF::get_accel_bias() const
{
    return Vector3f(
        x[10],
        x[11],
        x[12]
    );
}


// ============================================================================
// Gyro bias getter
// ============================================================================

Vector3f OurEKF::get_gyro_bias() const
{
    return Vector3f(
        x[13],
        x[14],
        x[15]
    );
}


// ============================================================================
// Individual getters
// ============================================================================

float OurEKF::get_position_x() const
{
    return x[0];
}

float OurEKF::get_position_y() const
{
    return x[1];
}

float OurEKF::get_position_z() const
{
    return x[2];
}

float OurEKF::get_velocity_x() const
{
    return x[3];
}

float OurEKF::get_velocity_y() const
{
    return x[4];
}

float OurEKF::get_velocity_z() const
{
    return x[5];
}

float OurEKF::get_accel_bias_x() const
{
    return x[10];
}

float OurEKF::get_accel_bias_y() const
{
    return x[11];
}

float OurEKF::get_accel_bias_z() const
{
    return x[12];
}

float OurEKF::get_gyro_bias_x() const
{
    return x[13];
}

float OurEKF::get_gyro_bias_y() const
{
    return x[14];
}

float OurEKF::get_gyro_bias_z() const
{
    return x[15];
}

float OurEKF::get_roll() const
{
    return roll;
}

float OurEKF::get_pitch() const
{
    return pitch;
}

float OurEKF::get_yaw() const
{
    return yaw;
}


// ============================================================================
// Get complete state
// ============================================================================

void OurEKF::get_state(float state[STATE_SIZE]) const
{
    for (int i = 0; i < STATE_SIZE; i++) {
        state[i] = x[i];
    }
}


// ============================================================================
// Get covariance
// ============================================================================

void OurEKF::get_covariance(
    float covariance[STATE_SIZE][STATE_SIZE]) const
{
    for (int i = 0; i < STATE_SIZE; i++) {

        for (int j = 0; j < STATE_SIZE; j++) {

            covariance[i][j] = P[i][j];
        }
    }
}


// ============================================================================
// Innovation getters
// ============================================================================

float OurEKF::get_innovation_x() const
{
    return innovation[0];
}

float OurEKF::get_innovation_y() const
{
    return innovation[1];
}

float OurEKF::get_innovation_z() const
{
    return innovation[2];
}


// ============================================================================
// Is initialized?
// ============================================================================

bool OurEKF::is_initialized() const
{
    return initialized;
}


// ============================================================================
// Matrix helpers
// ============================================================================

void OurEKF::zero_matrix_16(float A[16][16])
{
    memset(A, 0, sizeof(float) * 16 * 16);
}

void OurEKF::zero_matrix_16x3(float A[16][3])
{
    memset(A, 0, sizeof(float) * 16 * 3);
}

void OurEKF::zero_matrix_3x16(float A[3][16])
{
    memset(A, 0, sizeof(float) * 3 * 16);
}

void OurEKF::zero_matrix_3(float A[3][3])
{
    memset(A, 0, sizeof(float) * 3 * 3);
}

void OurEKF::identity_matrix_16(float A[16][16])
{
    zero_matrix_16(A);

    for (int i = 0; i < 16; i++) {
        A[i][i] = 1.0f;
    }
}


// ============================================================================
// 3x3 matrix inverse
//
// A^-1 = adj(A) / det(A)
//
// Returns false if determinant is too close to zero.
// ============================================================================

bool OurEKF::inverse_3x3(const float A[3][3],
                         float A_inv[3][3]) const
{
    float det =
          A[0][0] * (A[1][1] * A[2][2] -
                     A[1][2] * A[2][1])

        - A[0][1] * (A[1][0] * A[2][2] -
                     A[1][2] * A[2][0])

        + A[0][2] * (A[1][0] * A[2][1] -
                     A[1][1] * A[2][0]);


    if (fabsf(det) < 1.0e-9f) {
        return false;
    }


    float inv_det = 1.0f / det;


    A_inv[0][0] =
        (A[1][1] * A[2][2] -
         A[1][2] * A[2][1]) * inv_det;

    A_inv[0][1] =
        (A[0][2] * A[2][1] -
         A[0][1] * A[2][2]) * inv_det;

    A_inv[0][2] =
        (A[0][1] * A[1][2] -
         A[0][2] * A[1][1]) * inv_det;


    A_inv[1][0] =
        (A[1][2] * A[2][0] -
         A[1][0] * A[2][2]) * inv_det;

    A_inv[1][1] =
        (A[0][0] * A[2][2] -
         A[0][2] * A[2][0]) * inv_det;

    A_inv[1][2] =
        (A[0][2] * A[1][0] -
         A[0][0] * A[1][2]) * inv_det;


    A_inv[2][0] =
        (A[1][0] * A[2][1] -
         A[1][1] * A[2][0]) * inv_det;

    A_inv[2][1] =
        (A[0][1] * A[2][0] -
         A[0][0] * A[2][1]) * inv_det;

    A_inv[2][2] =
        (A[0][0] * A[1][1] -
         A[0][1] * A[1][0]) * inv_det;

    return true;
}


Vector3f OurEKF::get_accel_body() const
{
    return last_accel_body;
}

Vector3f OurEKF::get_accel_world() const
{
    return last_accel_world;
}




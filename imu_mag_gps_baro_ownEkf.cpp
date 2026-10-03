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

    // Barometer
    baro_noise_std = 0.5f;
    have_baro_offset = false;
    baro_offset = 0.0f;
    baro_calibrating = true;
    baro_calib_count = 0;
    baro_calib_sum = 0.0f;

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

    // Start a fresh bench-zero calibration after every EKF init.
    have_baro_offset = false;
    baro_offset = 0.0f;
    baro_calibrating = true;
    baro_calib_count = 0;
    baro_calib_sum = 0.0f;

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
    last_baro_sample_ms = 0;
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

    // Barometer correction. AP_Baro supplies altitude in metres.
    // update_baro() establishes a bench-zero reference from the first
    // valid samples and then fuses relative altitude into x[2] (NED Z).
    //update_baro(AP::baro().get_altitude());

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

uint8_t OurEKF::get_baro_fusion_result() const
{
    return static_cast<uint8_t>(baro_fusion_result);
}

float OurEKF::get_baro_innovation() const
{
    return baro_innovation;
}

float OurEKF::get_baro_raw() const
{
    return baro_raw;
}

float OurEKF::get_baro_altitude() const
{
    if (!have_baro_offset) {
        return 0.0f;
    }

    return -(x[2]);
}

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

float OurEKF::get_position_variance_z() const
{
    return P[2][2];
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




constexpr float BARO_GATE_SIGMA = 5.0f;


// bool OurEKF::update_baro(float baro_alt_raw)
// {
//     if (!initialized || !isfinite(baro_alt_raw)) {
//         return false;
//     }

//     // ------------------------------------------------------------
//     // Bench-zero calibration
//     // ------------------------------------------------------------
//     // Average the first 100 valid samples while the vehicle is
//     // stationary on the bench. This prevents one noisy barometer
//     // sample from becoming the zero reference.
//     constexpr uint16_t BARO_ZERO_SAMPLES = 100;

//     if (baro_calibrating) {
//         baro_calib_sum += baro_alt_raw;
//         baro_calib_count++;

//         if (baro_calib_count >= BARO_ZERO_SAMPLES) {
//             baro_offset = baro_calib_sum /
//                           static_cast<float>(baro_calib_count);
//             have_baro_offset = true;
//             baro_calibrating = false;
//         }

//         return true;
//     }

//     if (!have_baro_offset) {
//         return false;
//     }

//     // Barometer altitude is positive UP.
//     // EKF x[2] is NED Z, so positive DOWN.
//     const float baro_height_up = baro_alt_raw - baro_offset;
//     const float baro_z_ned = -baro_height_up;

//     // Measurement model:
//     //     z = -x[2]
//     // Therefore H[2] = -1.
//     const float innov = baro_z_ned - (-x[2]);

//     float Hrow[STATE_SIZE];
//     for (int i = 0; i < STATE_SIZE; i++) {
//         Hrow[i] = 0.0f;
//     }
//     Hrow[2] = -1.0f;

//     const float r_var = baro_noise_std * baro_noise_std;

//     const FuseResult result =
//         fuse_scalar(Hrow, innov, r_var, BARO_GATE_SIGMA);

//     return result == FUSE_OK;
// }


bool OurEKF::update_baro(float baro_alt_raw)
{
     if (!initialized || !isfinite(baro_alt_raw)) {
        return false;
    }

    const uint32_t sample_ms = AP::baro().get_last_update();
    if (sample_ms == last_baro_sample_ms) {
        return false;
    }
    last_baro_sample_ms = sample_ms;

    // ---------------------------------------------------------
    // First establish table/ground as zero
    // ---------------------------------------------------------

    if (!have_baro_offset) {

        // Collect samples while Cube is stationary on the table
        baro_calib_sum += baro_alt_raw;
        baro_calib_count++;

        // Average 100 samples
        if (baro_calib_count >= 100) {

            baro_offset =
                baro_calib_sum /
                static_cast<float>(baro_calib_count);

            have_baro_offset = true;
            baro_calibrating = false;

            // Force EKF Z position to zero at calibration point
            x[2] = 0.0f;

            return true;
        }

        // Do NOT fuse barometer before calibration completes
        return false;
    }

    // ---------------------------------------------------------
    // Relative altitude from the table
    // ---------------------------------------------------------

    const float altitude_up =
        baro_alt_raw - baro_offset;

    // EKF uses NED:
    //
    //     +Z = DOWN
    //     -Z = UP
    //
    const float baro_z_ned =
        -altitude_up;

    // ---------------------------------------------------------
    // Measurement:
    //
    // z = x[2]
    //
    // H = [0 0 1 0 ...]
    // ---------------------------------------------------------

    float Hrow[STATE_SIZE] = {};

    Hrow[2] = 1.0f;

    // innovation = measurement - predicted
    //
    // measurement = baro_z_ned
    // predicted   = x[2]

    const float innov =
        baro_z_ned - x[2];

    // ---------------------------------------------------------
    // Barometer measurement noise
    // ---------------------------------------------------------

    const float R =
        baro_noise_std * baro_noise_std;

    // ---------------------------------------------------------
    // Fuse scalar measurement
    // ---------------------------------------------------------

    const FuseResult result =
        fuse_scalar(
            Hrow,
            innov,
            R,
            5.0f
        );

    baro_raw=baro_alt_raw;
    baro_innovation=innov;
    
    return result == FUSE_OK;


    
}

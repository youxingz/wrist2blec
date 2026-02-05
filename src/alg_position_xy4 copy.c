// #define ENABLE_IT
#ifdef ENABLE_IT
/**
 * XY 平面运动，但有积分漂移，重力成分未完全消除
 */

#include "inc/alg_posture.h"
#include "inc/alg_position.h"

#include <errno.h>
#include <math.h>
#include <stdbool.h>
#include <string.h>


// ------------------------------------------------------------
// Debug switch (you can hook your own logger via ALG_DEBUG_PRINTF)
#define ENABLE_DEBUG_LOG
#ifdef ENABLE_DEBUG_LOG
#ifndef ALG_DEBUG_PRINTF
#define ALG_DEBUG_PRINTF(...) printk(__VA_ARGS__)
#endif
#else
#ifndef ALG_DEBUG_PRINTF
#define ALG_DEBUG_PRINTF(...) do { (void)0; } while (0)
#endif
#endif

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

// ------------------------------------------------------------
// Notes / Assumptions (matches your previous implementation):
// - imu_data_t:
//     accel: mg (1 g = 1000 mg)
//     gyro : dps (deg/s)
//     mag  : any linear unit (uT or raw), only direction is used (normalized)
// - Output world_posture_t:
//     yaw/pitch/roll: degrees
//     ax/ay/az: linear acceleration in BODY frame, unit m/s^2 (gravity removed + bias compensated)
// - Position integration (alg_position_update):
//     uses original accel (mg) + current quaternion to rotate to world,
//     integrates to get short-term relative displacement (meters).
// ------------------------------------------------------------

// posture config is provided by inc/alg_posture.h (kept identical API)
static volatile alg_posture_config_t posture_config;

// ------------------------ internal state ------------------------
typedef struct {
  // quaternion (world<-body) as used in Mahony (unit)
  float q0, q1, q2, q3;

  // integral feedback for Mahony
  float integral_fb_x;
  float integral_fb_y;
  float integral_fb_z;

  // filtered sensor values (same unit as input)
  float ax_f, ay_f, az_f;
  float gx_f, gy_f, gz_f;
  float mx_f, my_f, mz_f;
  bool filter_valid;

  // estimated gravity direction in BODY (unit vector), derived from quaternion
  float gbx, gby, gbz;

  // linear accel bias estimate in BODY (m/s^2), adapted when stationary
  float abx, aby, abz;

  // output cache
  world_posture_t current;
  bool valid;
} posture_state_t;

static posture_state_t state = {
  .q0 = 1.0f,
};

// position state (used by alg_position_update)
typedef struct {
  float x, y, z;       // m
  float vx, vy, vz;    // m/s

  // World-frame linear-acc bias estimate (m/s^2). This is a *slow* LPF of a_lin_world.
  // It is updated always, and faster when stationary, to suppress DC drift that otherwise
  // integrates into a straight-line runaway.
  float bx, by, bz;

  // World-frame linear-acc filtered (m/s^2) used for integration (after bias removal).
  float ax_f, ay_f, az_f;

  // gravity magnitude reference in mg (normally ~1000)

  // Slow DC remover mean on DEVICE X/Y (m/s^2)
  float mean_ax, mean_ay;
  float g_ref_mg;

  world_posture_t last;
  bool valid;
} position_state_t;

static position_state_t pos_state = {
  .g_ref_mg = 1000.0f,
};

// ------------------------ small helpers ------------------------
static inline bool is_finite3(float x, float y, float z)
{
  return isfinite(x) && isfinite(y) && isfinite(z);
}

static inline float lpf_update(float prev, float input, float alpha)
{
  return prev + alpha * (input - prev);
}

static inline float inv_sqrtf(float x)
{
  // safe inverse sqrt
  if (!isfinite(x) || x <= 0.0f) {
    return 0.0f;
  }
  return 1.0f / sqrtf(x);
}

static inline float norm3(float x, float y, float z)
{
  return sqrtf(x * x + y * y + z * z);
}

static inline float angle_diff_deg(float a, float b)
{
  float d = a - b;
  while (d > 180.0f) {
    d -= 360.0f;
  }
  while (d < -180.0f) {
    d += 360.0f;
  }
  return d;
}

// quaternion normalization
static inline void quat_normalize(posture_state_t *s)
{
  float n = s->q0 * s->q0 + s->q1 * s->q1 + s->q2 * s->q2 + s->q3 * s->q3;
  float inv = inv_sqrtf(n);
  if (inv > 0.0f) {
    s->q0 *= inv;
    s->q1 *= inv;
    s->q2 *= inv;
    s->q3 *= inv;
  } else {
    // fallback to identity if broken
    s->q0 = 1.0f;
    s->q1 = s->q2 = s->q3 = 0.0f;
  }
}

// gravity direction in BODY from quaternion (unit vector)
// matches typical AHRS derivation (assuming world Z is "up")
static inline void gravity_body_from_quat(posture_state_t *s, float *gbx, float *gby, float *gbz)
{
  // Using quaternion (q0,q1,q2,q3) where q rotates body -> world.
  // Gravity direction in body is third column of Rwb (world->body) applied to world Z.
  // Derived commonly as:
  // gbx = 2*(q1*q3 - q0*q2)
  // gby = 2*(q0*q1 + q2*q3)
  // gbz = q0^2 - q1^2 - q2^2 + q3^2
  float q0 = s->q0, q1 = s->q1, q2 = s->q2, q3 = s->q3;
  float x = 2.0f * (q1 * q3 - q0 * q2);
  float y = 2.0f * (q0 * q1 + q2 * q3);
  float z = q0 * q0 - q1 * q1 - q2 * q2 + q3 * q3;

  // normalize for safety
  float inv = inv_sqrtf(x * x + y * y + z * z);
  if (inv > 0.0f) {
    x *= inv;
    y *= inv;
    z *= inv;
  }
  *gbx = x;
  *gby = y;
  *gbz = z;
}

// Body->World rotation for a BODY vector using quaternion
static inline void rotate_body_to_world(const posture_state_t *s,
                                        float bx, float by, float bz,
                                        float *wx, float *wy, float *wz)
{
  // v_w = q * v_b * q_conj
  float q0 = s->q0, q1 = s->q1, q2 = s->q2, q3 = s->q3;

  // t = 2 * cross(q_vec, v)
  float tx = 2.0f * (q2 * bz - q3 * by);
  float ty = 2.0f * (q3 * bx - q1 * bz);
  float tz = 2.0f * (q1 * by - q2 * bx);

  // v' = v + q0*t + cross(q_vec, t)
  float cx = q2 * tz - q3 * ty;
  float cy = q3 * tx - q1 * tz;
  float cz = q1 * ty - q2 * tx;

  *wx = bx + q0 * tx + cx;
  *wy = by + q0 * ty + cy;
  *wz = bz + q0 * tz + cz;
}

static inline void rotate_world_to_body(const posture_state_t *s,
                                        float wx, float wy, float wz,
                                        float *bx, float *by, float *bz)
{
  // inverse rotation (BODY<-WORLD) for unit quaternion: use conjugate
  // v_b = q_conj * v_w * q
  float q0 = s->q0, q1 = -s->q1, q2 = -s->q2, q3 = -s->q3;

  float tx = 2.0f * (q2 * wz - q3 * wy);
  float ty = 2.0f * (q3 * wx - q1 * wz);
  float tz = 2.0f * (q1 * wy - q2 * wx);

  float cx = q2 * tz - q3 * ty;
  float cy = q3 * tx - q1 * tz;
  float cz = q1 * ty - q2 * tx;

  *bx = wx + q0 * tx + cx;
  *by = wy + q0 * ty + cy;
  *bz = wz + q0 * tz + cz;
}

// Quaternion -> Euler (yaw, pitch, roll) in degrees
static inline void quat_to_euler_deg(const posture_state_t *s, float *yaw, float *pitch, float *roll)
{
  float q0 = s->q0, q1 = s->q1, q2 = s->q2, q3 = s->q3;

  // yaw (Z)
  float ys = 2.0f * (q0 * q3 + q1 * q2);
  float yc = 1.0f - 2.0f * (q2 * q2 + q3 * q3);
  float y = atan2f(ys, yc);

  // pitch (Y)
  float ps = 2.0f * (q0 * q2 - q3 * q1);
  ps = fminf(fmaxf(ps, -1.0f), 1.0f);
  float p = asinf(ps);

  // roll (X)
  float rs = 2.0f * (q0 * q1 + q2 * q3);
  float rc = 1.0f - 2.0f * (q1 * q1 + q2 * q2);
  float r = atan2f(rs, rc);

  const float rad2deg = 180.0f / (float)M_PI;
  *yaw = y * rad2deg;
  *pitch = p * rad2deg;
  *roll = r * rad2deg;
}

// ------------------------ Mahony AHRS ------------------------
// This implementation is intentionally conservative and stable for motion analysis:
// - always normalizes accel/mag (so scale doesn't matter)
// - integrates gyro (rad/s)
// - uses accel+mag correction when valid
static void mahony_update(posture_state_t *s,
                          float gx_rps, float gy_rps, float gz_rps,
                          float ax_mg, float ay_mg, float az_mg,
                          float mx, float my, float mz,
                          float dt)
{
  // gains (tune as you like)
  // larger Kp => faster correction but more noise
  const float Kp = 2.0f;
  const float Ki = 0.05f;

  // normalize accelerometer
  float ax = ax_mg, ay = ay_mg, az = az_mg;
  float an = ax * ax + ay * ay + az * az;
  float inv_an = inv_sqrtf(an);
  bool acc_ok = (inv_an > 0.0f);
  if (acc_ok) {
    ax *= inv_an;
    ay *= inv_an;
    az *= inv_an;
  }

  // normalize magnetometer (optional)
  float mxn = mx, myn = my, mzn = mz;
  float mn = mxn * mxn + myn * myn + mzn * mzn;
  float inv_mn = inv_sqrtf(mn);
  bool mag_ok = (inv_mn > 0.0f);
  if (mag_ok) {
    mxn *= inv_mn;
    myn *= inv_mn;
    mzn *= inv_mn;
  }

  float q0 = s->q0, q1 = s->q1, q2 = s->q2, q3 = s->q3;

  float ex = 0.0f, ey = 0.0f, ez = 0.0f;

  if (acc_ok) {
    // Estimated direction of gravity (from quaternion) in BODY coordinates
    // Same as gravity_body_from_quat, but we compute quickly here.
    float vx = 2.0f * (q1 * q3 - q0 * q2);
    float vy = 2.0f * (q0 * q1 + q2 * q3);
    float vz = q0 * q0 - q1 * q1 - q2 * q2 + q3 * q3;

    // Error is cross(acc_meas, gravity_est)
    ex += (ay * vz - az * vy);
    ey += (az * vx - ax * vz);
    ez += (ax * vy - ay * vx);
  }

  if (acc_ok && mag_ok) {
    // Reference direction of Earth's magnetic field
    float hx = 2.0f * (mxn * (0.5f - q2 * q2 - q3 * q3) +
                       myn * (q1 * q2 - q0 * q3) +
                       mzn * (q1 * q3 + q0 * q2));
    float hy = 2.0f * (mxn * (q1 * q2 + q0 * q3) +
                       myn * (0.5f - q1 * q1 - q3 * q3) +
                       mzn * (q2 * q3 - q0 * q1));
    float bx = sqrtf(hx * hx + hy * hy);
    float bz = 2.0f * (mxn * (q1 * q3 - q0 * q2) +
                       myn * (q2 * q3 + q0 * q1) +
                       mzn * (0.5f - q1 * q1 - q2 * q2));

    // Estimated direction of magnetic field
    float wx = 2.0f * (bx * (0.5f - q2 * q2 - q3 * q3) + bz * (q1 * q3 - q0 * q2));
    float wy = 2.0f * (bx * (q1 * q2 - q0 * q3) + bz * (q0 * q1 + q2 * q3));
    float wz = 2.0f * (bx * (q0 * q2 + q1 * q3) + bz * (0.5f - q1 * q1 - q2 * q2));

    // Error is cross(mag_meas, mag_est)
    ex += (myn * wz - mzn * wy);
    ey += (mzn * wx - mxn * wz);
    ez += (mxn * wy - myn * wx);
  }

  // Integral feedback
  if (Ki > 0.0f) {
    s->integral_fb_x += Ki * ex * dt;
    s->integral_fb_y += Ki * ey * dt;
    s->integral_fb_z += Ki * ez * dt;

    gx_rps += s->integral_fb_x;
    gy_rps += s->integral_fb_y;
    gz_rps += s->integral_fb_z;
  } else {
    s->integral_fb_x = 0.0f;
    s->integral_fb_y = 0.0f;
    s->integral_fb_z = 0.0f;
  }

  // Proportional feedback
  gx_rps += Kp * ex;
  gy_rps += Kp * ey;
  gz_rps += Kp * ez;

  // Integrate quaternion rate: q_dot = 0.5 * q ⊗ [0, gyro]
  float half_dt = 0.5f * dt;
  float qa = q0;
  float qb = q1;
  float qc = q2;
  float qd = q3;

  q0 += (-qb * gx_rps - qc * gy_rps - qd * gz_rps) * half_dt;
  q1 += (qa * gx_rps + qc * gz_rps - qd * gy_rps) * half_dt;
  q2 += (qa * gy_rps - qb * gz_rps + qd * gx_rps) * half_dt;
  q3 += (qa * gz_rps + qb * gy_rps - qc * gx_rps) * half_dt;

  s->q0 = q0;
  s->q1 = q1;
  s->q2 = q2;
  s->q3 = q3;
  quat_normalize(s);
}

// ------------------------ public APIs (MUST keep signatures) ------------------------
int alg_posture_init(alg_posture_config_t cfg)
{
  if (cfg.sample_us == 0) {
    cfg.sample_us = 2000; // default 500Hz
  }
  posture_config = cfg;

  memset(&state, 0, sizeof(state));
  state.q0 = 1.0f;
  state.valid = false;
  state.filter_valid = false;

  // keep position state too (safe default)
  memset(&pos_state, 0, sizeof(pos_state));
  pos_state.g_ref_mg = 1000.0f;

  return 0;
}

/**
 * Update posture at fixed sampling frequency.
 * Input units: accel mg, gyro dps, mag arbitrary.
 */
world_posture_t alg_posture_update(imu_data_t *data)
{
  if (!data) {
    return state.current;
  }

  float ax = data->ax;
  float ay = data->ay;
  float az = data->az;
  float gx = data->gx;
  float gy = data->gy;
  float gz = data->gz;
  float mx = data->mx;
  float my = data->my;
  float mz = data->mz;

  float dt = (float)posture_config.sample_us / 1000000.0f;
  if (!isfinite(dt) || dt <= 0.0f) {
    dt = 0.002f;
  }

  // 1st-order LPF for stability (cutoffs are conservative for motion analysis)
  const float acc_cutoff_hz = 10.0f;
  const float gyro_cutoff_hz = 20.0f;
  const float mag_cutoff_hz = 10.0f;

  const float tau_acc = 1.0f / (2.0f * (float)M_PI * acc_cutoff_hz);
  const float tau_gyro = 1.0f / (2.0f * (float)M_PI * gyro_cutoff_hz);
  const float tau_mag = 1.0f / (2.0f * (float)M_PI * mag_cutoff_hz);

  const float alpha_acc = dt / (tau_acc + dt);
  const float alpha_gyro = dt / (tau_gyro + dt);
  const float alpha_mag = dt / (tau_mag + dt);

  if (!state.filter_valid) {
    state.ax_f = ax;
    state.ay_f = ay;
    state.az_f = az;
    state.gx_f = gx;
    state.gy_f = gy;
    state.gz_f = gz;
    state.mx_f = mx;
    state.my_f = my;
    state.mz_f = mz;
    state.filter_valid = true;
  } else {
    if (is_finite3(ax, ay, az)) {
      state.ax_f = lpf_update(state.ax_f, ax, alpha_acc);
      state.ay_f = lpf_update(state.ay_f, ay, alpha_acc);
      state.az_f = lpf_update(state.az_f, az, alpha_acc);
    }
    if (is_finite3(gx, gy, gz)) {
      state.gx_f = lpf_update(state.gx_f, gx, alpha_gyro);
      state.gy_f = lpf_update(state.gy_f, gy, alpha_gyro);
      state.gz_f = lpf_update(state.gz_f, gz, alpha_gyro);
    }
    if (is_finite3(mx, my, mz)) {
      state.mx_f = lpf_update(state.mx_f, mx, alpha_mag);
      state.my_f = lpf_update(state.my_f, my, alpha_mag);
      state.mz_f = lpf_update(state.mz_f, mz, alpha_mag);
    }
  }

  ax = state.ax_f;
  ay = state.ay_f;
  az = state.az_f;
  gx = state.gx_f;
  gy = state.gy_f;
  gz = state.gz_f;
  mx = state.mx_f;
  my = state.my_f;
  mz = state.mz_f;

  // gyro norm in dps for stationary detect
  float gyro_norm_dps = norm3(gx, gy, gz);

  // convert gyro dps -> rad/s
  const float deg2rad = (float)M_PI / 180.0f;
  float gx_rps = gx * deg2rad;
  float gy_rps = gy * deg2rad;
  float gz_rps = gz * deg2rad;

  mahony_update(&state, gx_rps, gy_rps, gz_rps, ax, ay, az, mx, my, mz, dt);

  // gravity direction in body (unit)
  gravity_body_from_quat(&state, &state.gbx, &state.gby, &state.gbz);

  // linear accel in BODY (remove gravity using gravity direction)
  const float g_ref_mg = 1000.0f;           // 1g in mg
  const float mg_to_mps2 = 0.00980665f;
  float lin_bx = (ax - state.gbx * g_ref_mg) * mg_to_mps2;
  float lin_by = (ay - state.gby * g_ref_mg) * mg_to_mps2;
  float lin_bz = (az - state.gbz * g_ref_mg) * mg_to_mps2;

  // stationary detect: small gyro + small linear accel
  float lin_norm = norm3(lin_bx, lin_by, lin_bz);
  bool stationary = (gyro_norm_dps < 2.0f) && (lin_norm < 0.25f);

  // bias adaptation in BODY when stationary
  if (stationary) {
    const float bias_alpha = 0.01f;
    state.abx = lpf_update(state.abx, lin_bx, bias_alpha);
    state.aby = lpf_update(state.aby, lin_by, bias_alpha);
    state.abz = lpf_update(state.abz, lin_bz, bias_alpha);
  }

  // remove bias
  lin_bx -= state.abx;
  lin_by -= state.aby;
  lin_bz -= state.abz;

  state.current.ax = lin_bx;
  state.current.ay = lin_by;
  state.current.az = lin_bz;

  quat_to_euler_deg(&state, &state.current.yaw, &state.current.pitch, &state.current.roll);
  state.valid = true;

  return state.current;
}

world_position_t alg_position_update(const world_posture_t *posture,
                                     const imu_data_t *imu)
{
  world_position_t out = {
    .x = pos_state.x,
    .y = pos_state.y,
    .z = pos_state.z,
    .yaw = pos_state.last.yaw,
    .pitch = pos_state.last.pitch,
    .roll = pos_state.last.roll,
  };

  if (!posture || !imu) {
    return out;
  }

  world_posture_t p = *posture;
  out.yaw = p.yaw;
  out.pitch = p.pitch;
  out.roll = p.roll;

  if (!pos_state.valid) {
    pos_state.last = p;
    pos_state.valid = true;
    // reset filters
    pos_state.bx = pos_state.by = pos_state.bz = 0.0f;
    pos_state.ax_f = pos_state.ay_f = pos_state.az_f = 0.0f;
    pos_state.mean_ax = pos_state.mean_ay = 0.0f;
    pos_state.vx = pos_state.vy = pos_state.vz = 0.0f;
    pos_state.x = pos_state.y = pos_state.z = 0.0f;
    return out;
  }

#ifndef ALG_POSITION_SAMPLE_US
#define ALG_POSITION_SAMPLE_US 10000U
#endif

  // Fixed-rate integration. If you have real timestamps, replace this with measured dt.
  float dt = (float)ALG_POSITION_SAMPLE_US / 1000000.0f;
  if (!isfinite(dt) || dt <= 0.0f) {
    dt = 0.01f;
  }
  // clamp dt against occasional spikes
  if (dt < 0.0005f) dt = 0.0005f;
  if (dt > 0.05f)   dt = 0.05f;

  // raw accel in BODY (mg) - this is specific force (includes gravity)
  float ax_mg = imu->ax;
  float ay_mg = imu->ay;
  float az_mg = imu->az;
  if (!is_finite3(ax_mg, ay_mg, az_mg) || !state.valid) {
    pos_state.last = p;
    return out;
  }

  // ------------------------------------------------------------
  // Step 1) Remove gravity in BODY using gravity direction estimated by posture quaternion.
  //         (imu provides specific force; at rest |a| ≈ 1000 mg)
  // ------------------------------------------------------------
  float lin_bx_mg = ax_mg - state.gbx * pos_state.g_ref_mg;
  float lin_by_mg = ay_mg - state.gby * pos_state.g_ref_mg;
  float lin_bz_mg = az_mg - state.gbz * pos_state.g_ref_mg;

  // Convert BODY linear accel (still mg) -> m/s^2
// NOTE: Output requested is in DEVICE(BODY) XY, so we keep everything in BODY frame.
const float mg_to_mps2 = 0.00980665f;
float lin_bx = lin_bx_mg * mg_to_mps2;
float lin_by = lin_by_mg * mg_to_mps2;
float lin_bz = lin_bz_mg * mg_to_mps2;

  // ------------------------------------------------------------
  // OUTPUT DEFINITION (per your latest spec):
  //   - 2D displacement in DEVICE(BODY) X-Y plane.
  //   - Ignore motion along DEVICE Z axis (treat as lateral/sideways).
  // In world terms: forward/back (X) and up/down (Y) are kept, left/right (Z) ignored.
  // We still keep lin_bz for stationary detection (to avoid ZUPT while moving sideways),
  // but we do NOT integrate it into position.

  // Optional: clamp extreme spikes (helps with occasional glitches)
  {
    float n = norm3(lin_bx, lin_by, lin_bz);
    const float n_max = 30.0f; // ~3 g
    if (isfinite(n) && n > n_max && n > 0.0f) {
      float s = n_max / n;
      lin_bx *= s;
      lin_by *= s;
      lin_bz *= s;
    }
  }

  // ------------------------------------------------------------
  // Step 2) Stationary detect (cheap, but effective): posture delta + world linear accel norm
  // ------------------------------------------------------------
  float dy = angle_diff_deg(p.yaw, pos_state.last.yaw);
  float dp = angle_diff_deg(p.pitch, pos_state.last.pitch);
  float dr = angle_diff_deg(p.roll, pos_state.last.roll);
  float max_d = fmaxf(fmaxf(fabsf(dy), fabsf(dp)), fabsf(dr));
  float lin_norm = norm3(lin_bx, lin_by, lin_bz);

  const float stationary_angle_deg = 1.0f;
  const float stationary_lin_mps2 = 0.25f;
  bool stationary = (max_d < stationary_angle_deg) && (lin_norm < stationary_lin_mps2);

  // ------------------------------------------------------------
  // Step 3) Bias tracking in WORLD (key improvement):
  //   - A tiny constant error (bias or gravity projection) will integrate into a runaway line.
  //   - We track and subtract a slow-varying "DC" component from a_lin_world.
  //   - When stationary, adapt faster and also do ZUPT.
  // ------------------------------------------------------------
  float tau_bias_s = stationary ? 0.35f : 2.0f; // stationary: fast; moving: slow
  if (tau_bias_s < 0.05f) tau_bias_s = 0.05f;
  float alpha_bias = dt / (tau_bias_s + dt);

  pos_state.bx = lpf_update(pos_state.bx, lin_bx, alpha_bias);
  pos_state.by = lpf_update(pos_state.by, lin_by, alpha_bias);
  pos_state.bz = lpf_update(pos_state.bz, lin_bz, alpha_bias);

  // Subtract bias (high-pass effect)
  lin_bx -= pos_state.bx;
  lin_by -= pos_state.by;
  /* lin_bz kept for stationarity only; ignore in integration */
  (void)lin_bz;


  // ------------------------------------------------------------
  // Step 4) Low-pass linear accel (makes trajectory "presentable")
  // ------------------------------------------------------------
  const float fc_acc_hz = 8.0f;
  float tau_acc = 1.0f / (2.0f * (float)M_PI * fc_acc_hz);
  float alpha_acc = dt / (tau_acc + dt);

  pos_state.ax_f = lpf_update(pos_state.ax_f, lin_bx, alpha_acc);
  pos_state.ay_f = lpf_update(pos_state.ay_f, lin_by, alpha_acc);
  pos_state.az_f = 0.0f; // ignore DEVICE Z axis for 2D XY output


  // ------------------------------------------------------------
  // Step 4.5) DC remover on DEVICE X/Y (key for removing "constant pull" -> spiral)
  //   - Track a slow-varying mean and subtract it before integration.
  //   - When stationary, converge faster so we don't get a "kick" when moving again.
  // ------------------------------------------------------------
  float tau_mean_s = stationary ? 0.30f : 2.0f; // seconds
  float alpha_mean = dt / (tau_mean_s + dt);
  pos_state.mean_ax = lpf_update(pos_state.mean_ax, pos_state.ax_f, alpha_mean);
  pos_state.mean_ay = lpf_update(pos_state.mean_ay, pos_state.ay_f, alpha_mean);

  float ax_use = pos_state.ax_f - pos_state.mean_ax;
  float ay_use = pos_state.ay_f - pos_state.mean_ay;

#ifdef ENABLE_DEBUG_LOG
  // You can hook this macro to your logger if needed.
  ALG_DEBUG_PRINTF("acc_use=%.3f %.3f\n", ax_use, ay_use);
#endif

  // ------------------------------------------------------------
  // Step 5) ZUPT + gravity magnitude update when stationary
  // ------------------------------------------------------------
  if (stationary) {
    pos_state.vx = 0.0f;
    pos_state.vy = 0.0f;
    pos_state.vz = 0.0f;

    // update gravity magnitude reference (mg) slowly using raw accel magnitude
    const float g_alpha = 0.02f;
    float acc_norm_mg = norm3(ax_mg, ay_mg, az_mg);
    if (isfinite(acc_norm_mg) && acc_norm_mg > 100.0f) {
      pos_state.g_ref_mg = lpf_update(pos_state.g_ref_mg, acc_norm_mg, g_alpha);
    }

    // also pull accel filter toward 0 to avoid a "kick" when leaving stationary
    pos_state.ax_f *= 0.5f;
    pos_state.ay_f *= 0.5f;
    pos_state.az_f *= 0.5f;

    // also pull DC mean toward current filtered accel (prevents spiral kick after ZUPT)
    pos_state.mean_ax = lpf_update(pos_state.mean_ax, pos_state.ax_f, 0.5f);
    pos_state.mean_ay = lpf_update(pos_state.mean_ay, pos_state.ay_f, 0.5f);
  }

  // ------------------------------------------------------------
  // Step 6) Integrate velocity/position
  // ------------------------------------------------------------
  pos_state.vx += ax_use * dt;
  pos_state.vy += ay_use * dt;
  // ignore device Z displacement
  pos_state.vz = 0.0f;
// velocity leak to keep trajectory bounded
  const float vel_leak = 0.08f; // stronger default to avoid runaway
  float k = 1.0f - vel_leak * dt;
  if (k < 0.0f) k = 0.0f;
  if (k > 1.0f) k = 1.0f;
  pos_state.vx *= k;
  pos_state.vy *= k;
  pos_state.vz *= k;

  pos_state.x += pos_state.vx * dt;
  pos_state.y += pos_state.vy * dt;
  pos_state.z = 0.0f;
pos_state.last = p;

  // Output requirement:
//   1) Only need 2D trajectory in DEVICE(BODY) coordinates (x/y).
//   2) Ignore motion along device Z axis: set output z = 0.
// We keep integrating in WORLD for stability, then convert the integrated
// position into BODY frame at the current posture.
out.x = pos_state.x;
  out.y = pos_state.y;
  out.z = 0.0f;
  return out;
}

bool alg_posture_is_yaw_balanced(void)
{
  if (!state.valid) {
    return false;
  }
  if (posture_config.balance_yaw_thresh < 0.0f) {
    return true;
  }
  float dy = angle_diff_deg(state.current.yaw, posture_config.balance_yaw);
  return fabsf(dy) <= posture_config.balance_yaw_thresh;
}

bool alg_posture_is_pitch_balanced(void)
{
  if (!state.valid) {
    return false;
  }
  if (posture_config.balance_pitch_thresh < 0.0f) {
    return true;
  }
  float dp = angle_diff_deg(state.current.pitch, posture_config.balance_pitch);
  return fabsf(dp) <= posture_config.balance_pitch_thresh;
}

bool alg_posture_is_roll_balanced(void)
{
  if (!state.valid) {
    return false;
  }
  if (posture_config.balance_roll_thresh < 0.0f) {
    return true;
  }
  float dr = angle_diff_deg(state.current.roll, posture_config.balance_roll);
  return fabsf(dr) <= posture_config.balance_roll_thresh;
}

int alg_posture_update_threshold(uint8_t axis, float threshold_deg)
{
  switch (axis) {
    case ALG_POSTURE_AXIS_YAW:
      posture_config.balance_yaw_thresh = threshold_deg;
      break;
    case ALG_POSTURE_AXIS_PITCH:
      posture_config.balance_pitch_thresh = threshold_deg;
      break;
    case ALG_POSTURE_AXIS_ROLL:
      posture_config.balance_roll_thresh = threshold_deg;
      break;
    default:
      return -EINVAL;
  }
  return 0;
}

#endif

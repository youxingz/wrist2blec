#include "inc/alg.h"

#include <errno.h>
#include <math.h>
#include <stdbool.h>
#include <string.h>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

static volatile alg_config_t config;

typedef struct {
  float q0;
  float q1;
  float q2;
  float q3;
  float integral_fb_x;
  float integral_fb_y;
  float integral_fb_z;
  float ax_f;
  float ay_f;
  float az_f;
  float gx_f;
  float gy_f;
  float gz_f;
  float mx_f;
  float my_f;
  float mz_f;
  bool filter_valid;
  float abx;
  float aby;
  float abz;
  world_pos_t current;
  bool valid;
} alg_state_t;

static alg_state_t state = {
  .q0 = 1.0f,
  .q1 = 0.0f,
  .q2 = 0.0f,
  .q3 = 0.0f,
};

static inline bool is_finite3(float x, float y, float z)
{
  return isfinite(x) && isfinite(y) && isfinite(z);
}

static inline float inv_sqrt(float x)
{
  if (x <= 0.0f || !isfinite(x)) {
    return 0.0f;
  }
  return 1.0f / sqrtf(x);
}

static inline float norm3(float x, float y, float z)
{
  return sqrtf(x * x + y * y + z * z);
}

static inline float lpf_update(float prev, float input, float alpha)
{
  return prev + alpha * (input - prev);
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

static inline void gravity_body_from_quat(const alg_state_t *st, float *gx, float *gy, float *gz)
{
  float q0 = st->q0;
  float q1 = st->q1;
  float q2 = st->q2;
  float q3 = st->q3;
  *gx = 2.0f * (q1 * q3 - q0 * q2);
  *gy = 2.0f * (q0 * q1 + q2 * q3);
  *gz = q0 * q0 - q1 * q1 - q2 * q2 + q3 * q3;
}

static inline bool normalize3(float *x, float *y, float *z)
{
  float n2 = (*x) * (*x) + (*y) * (*y) + (*z) * (*z);
  float inv = inv_sqrt(n2);
  if (inv <= 0.0f) {
    return false;
  }
  *x *= inv;
  *y *= inv;
  *z *= inv;
  return true;
}

static inline void quat_normalize(alg_state_t *st)
{
  float inv = inv_sqrt(st->q0 * st->q0 + st->q1 * st->q1 + st->q2 * st->q2 + st->q3 * st->q3);
  if (inv <= 0.0f) {
    st->q0 = 1.0f;
    st->q1 = st->q2 = st->q3 = 0.0f;
    return;
  }
  st->q0 *= inv;
  st->q1 *= inv;
  st->q2 *= inv;
  st->q3 *= inv;
}

static inline void quat_to_euler(const alg_state_t *st, float *yaw, float *pitch, float *roll)
{
  float q0 = st->q0;
  float q1 = st->q1;
  float q2 = st->q2;
  float q3 = st->q3;

  float sinr_cosp = 2.0f * (q0 * q1 + q2 * q3);
  float cosr_cosp = 1.0f - 2.0f * (q1 * q1 + q2 * q2);
  float r = atan2f(sinr_cosp, cosr_cosp);

  float sinp = 2.0f * (q0 * q2 - q3 * q1);
  float p;
  if (fabsf(sinp) >= 1.0f) {
    p = copysignf((float)M_PI / 2.0f, sinp);
  } else {
    p = asinf(sinp);
  }

  float siny_cosp = 2.0f * (q0 * q3 + q1 * q2);
  float cosy_cosp = 1.0f - 2.0f * (q2 * q2 + q3 * q3);
  float y = atan2f(siny_cosp, cosy_cosp);

  // 输出单位：deg
  const float rad2deg = 57.2957795f;
  *yaw = y * rad2deg;
  *pitch = p * rad2deg;
  *roll = r * rad2deg;
}

static inline void rotate_body_to_world(const alg_state_t *st, float bx, float by, float bz,
                                        float *wx, float *wy, float *wz)
{
  float q0 = st->q0;
  float q1 = st->q1;
  float q2 = st->q2;
  float q3 = st->q3;

  *wx = (1.0f - 2.0f * (q2 * q2 + q3 * q3)) * bx +
        2.0f * (q1 * q2 - q0 * q3) * by +
        2.0f * (q1 * q3 + q0 * q2) * bz;
  *wy = 2.0f * (q1 * q2 + q0 * q3) * bx +
        (1.0f - 2.0f * (q1 * q1 + q3 * q3)) * by +
        2.0f * (q2 * q3 - q0 * q1) * bz;
  *wz = 2.0f * (q1 * q3 - q0 * q2) * bx +
        2.0f * (q2 * q3 + q0 * q1) * by +
        (1.0f - 2.0f * (q1 * q1 + q2 * q2)) * bz;
}

static void mahony_update(alg_state_t *st,
                          float gx, float gy, float gz,
                          float ax, float ay, float az,
                          float mx, float my, float mz,
                          float dt)
{
  // 参数：建议根据噪声/采样率再调优
  const float kp = 2.0f;
  const float ki = 0.01f;

  if (!is_finite3(ax, ay, az) || !is_finite3(gx, gy, gz)) {
    return;
  }

  bool acc_ok = normalize3(&ax, &ay, &az);
  if (!acc_ok) {
    return;
  }

  bool mag_ok = is_finite3(mx, my, mz) && normalize3(&mx, &my, &mz);

  // Estimated gravity direction
  float q0 = st->q0;
  float q1 = st->q1;
  float q2 = st->q2;
  float q3 = st->q3;

  float vx = 2.0f * (q1 * q3 - q0 * q2);
  float vy = 2.0f * (q0 * q1 + q2 * q3);
  float vz = q0 * q0 - q1 * q1 - q2 * q2 + q3 * q3;

  float ex = 0.0f;
  float ey = 0.0f;
  float ez = 0.0f;

  if (mag_ok) {
    // Reference direction of Earth's magnetic field
    float hx = 2.0f * mx * (0.5f - q2 * q2 - q3 * q3) +
               2.0f * my * (q1 * q2 - q0 * q3) +
               2.0f * mz * (q1 * q3 + q0 * q2);
    float hy = 2.0f * mx * (q1 * q2 + q0 * q3) +
               2.0f * my * (0.5f - q1 * q1 - q3 * q3) +
               2.0f * mz * (q2 * q3 - q0 * q1);
    float bx = sqrtf(hx * hx + hy * hy);
    float bz = 2.0f * mx * (q1 * q3 - q0 * q2) +
               2.0f * my * (q2 * q3 + q0 * q1) +
               2.0f * mz * (0.5f - q1 * q1 - q2 * q2);

    // Estimated direction of magnetic field
    float wx = 2.0f * bx * (0.5f - q2 * q2 - q3 * q3) +
               2.0f * bz * (q1 * q3 - q0 * q2);
    float wy = 2.0f * bx * (q1 * q2 - q0 * q3) +
               2.0f * bz * (q0 * q1 + q2 * q3);
    float wz = 2.0f * bx * (q0 * q2 + q1 * q3) +
               2.0f * bz * (0.5f - q1 * q1 - q2 * q2);

    ex = (ay * vz - az * vy) + (my * wz - mz * wy);
    ey = (az * vx - ax * vz) + (mz * wx - mx * wz);
    ez = (ax * vy - ay * vx) + (mx * wy - my * wx);
  } else {
    ex = (ay * vz - az * vy);
    ey = (az * vx - ax * vz);
    ez = (ax * vy - ay * vx);
  }

  if (ki > 0.0f && isfinite(dt) && dt > 0.0f) {
    st->integral_fb_x += ki * ex * dt;
    st->integral_fb_y += ki * ey * dt;
    st->integral_fb_z += ki * ez * dt;
  } else {
    st->integral_fb_x = 0.0f;
    st->integral_fb_y = 0.0f;
    st->integral_fb_z = 0.0f;
  }

  gx += kp * ex + st->integral_fb_x;
  gy += kp * ey + st->integral_fb_y;
  gz += kp * ez + st->integral_fb_z;

  // Integrate quaternion rate and normalize
  float half_dt = 0.5f * dt;
  float qa = q0;
  float qb = q1;
  float qc = q2;

  q0 += (-qb * gx - qc * gy - q3 * gz) * half_dt;
  q1 += (qa * gx + qc * gz - q3 * gy) * half_dt;
  q2 += (qa * gy - qb * gz + q3 * gx) * half_dt;
  q3 += (qa * gz + qb * gy - qc * gx) * half_dt;

  st->q0 = q0;
  st->q1 = q1;
  st->q2 = q2;
  st->q3 = q3;
  quat_normalize(st);
}

int alg_imu_init(alg_config_t cfg)
{
  if (cfg.sample_us == 0) {
    cfg.sample_us = 2000; // 默认 500Hz
  }
  config = cfg;

  memset(&state, 0, sizeof(state));
  state.q0 = 1.0f;
  state.valid = false;
  return 0;
}

/**
 * 以固定频率采集并更新至此
 */
int alg_imu_update(imu_data_t * data)
{
  if (!data) {
    return -EINVAL;
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

  float dt = (float)config.sample_us / 1000000.0f;
  if (!isfinite(dt) || dt <= 0.0f) {
    dt = 0.002f;
  }

  // 简单一阶低通，抑制高频抖动噪声
  // 截止频率可按实际噪声/响应再调整
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

  // 角速度幅值（dps），用于静止检测
  float gyro_norm_dps = norm3(gx, gy, gz);

  // 输入单位假设：加速度 mg，角速度 dps（来自驱动配置）
  const float deg2rad = 0.01745329252f;
  gx *= deg2rad;
  gy *= deg2rad;
  gz *= deg2rad;

  mahony_update(&state, gx, gy, gz, ax, ay, az, mx, my, mz, dt);

  // 线加速度（m/s^2），机体坐标系，去除重力分量
  const float g_ref_mg = 1000.0f;
  const float mg_to_mps2 = 0.00980665f;
  float gbx, gby, gbz;
  gravity_body_from_quat(&state, &gbx, &gby, &gbz);
  float lin_bx_mg = ax - gbx * g_ref_mg;
  float lin_by_mg = ay - gby * g_ref_mg;
  float lin_bz_mg = az - gbz * g_ref_mg;
  float lin_bx = lin_bx_mg * mg_to_mps2;
  float lin_by = lin_by_mg * mg_to_mps2;
  float lin_bz = lin_bz_mg * mg_to_mps2;

  // 简单零偏估计：静止时用低通估计线加速度偏置
  const float gyro_stationary_dps = 1.5f;
  const float lin_acc_stationary = 0.2f; // m/s^2
  const float acc_norm_stationary_mg = 50.0f;
  float acc_norm_mg = norm3(ax, ay, az);
  bool stationary = (gyro_norm_dps < gyro_stationary_dps) &&
                    (fabsf(acc_norm_mg - g_ref_mg) < acc_norm_stationary_mg) &&
                    (norm3(lin_bx, lin_by, lin_bz) < lin_acc_stationary);

  if (stationary && is_finite3(lin_bx, lin_by, lin_bz)) {
    const float bias_alpha = 0.01f;
    state.abx = lpf_update(state.abx, lin_bx, bias_alpha);
    state.aby = lpf_update(state.aby, lin_by, bias_alpha);
    state.abz = lpf_update(state.abz, lin_bz, bias_alpha);
  }

  // 去除估计偏置
  lin_bx -= state.abx;
  lin_by -= state.aby;
  lin_bz -= state.abz;

  state.current.ax = lin_bx;
  state.current.ay = lin_by;
  state.current.az = lin_bz;

  quat_to_euler(&state, &state.current.yaw, &state.current.pitch, &state.current.roll);
  state.valid = true;

  // if return 0, call: #alg_imu_get_current/1
  return 0;
}

/**
 * 获取解算出来的最新姿势（世界坐标）
 */
int alg_imu_get_current(world_pos_t * position)
{
  if (!position) {
    return -EINVAL;
  }
  if (!state.valid) {
    return -EAGAIN;
  }
  *position = state.current;
  return 0;
}

bool alg_imu_is_yaw_balanced(void)
{
  if (!state.valid) {
    return false;
  }
  if (config.balance_yaw_thresh < 0.0f) {
    return true;
  }
  float dy = angle_diff_deg(state.current.yaw, config.balance_yaw);
  return fabsf(dy) <= config.balance_yaw_thresh;
}

bool alg_imu_is_pitch_balanced(void)
{
  if (!state.valid) {
    return false;
  }
  if (config.balance_pitch_thresh < 0.0f) {
    return true;
  }
  float dp = angle_diff_deg(state.current.pitch, config.balance_pitch);
  return fabsf(dp) <= config.balance_pitch_thresh;
}

bool alg_imu_is_roll_balanced(void)
{
  if (!state.valid) {
    return false;
  }
  if (config.balance_roll_thresh < 0.0f) {
    return true;
  }
  float dr = angle_diff_deg(state.current.roll, config.balance_roll);
  return fabsf(dr) <= config.balance_roll_thresh;
}

int alg_update_threshold(uint8_t axis, float threshold_deg)
{
  switch (axis) {
    case ALG_AXIS_YAW:
      config.balance_yaw_thresh = threshold_deg;
      break;
    case ALG_AXIS_PITCH:
      config.balance_pitch_thresh = threshold_deg;
      break;
    case ALG_AXIS_ROLL:
      config.balance_roll_thresh = threshold_deg;
      break;
    default:
      return -EINVAL;
  }

  return 0;
}

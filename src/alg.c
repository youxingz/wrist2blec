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
  config.sample_us = cfg.sample_us;

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

  // 输入单位假设：加速度 mg，角速度 dps（来自驱动配置）
  const float deg2rad = 0.01745329252f;
  gx *= deg2rad;
  gy *= deg2rad;
  gz *= deg2rad;

  float dt = (float)config.sample_us / 1000000.0f;
  if (!isfinite(dt) || dt <= 0.0f) {
    dt = 0.002f;
  }

  mahony_update(&state, gx, gy, gz, ax, ay, az, mx, my, mz, dt);

  // 世界坐标线加速度（单位：mg），去除重力分量
  float wx, wy, wz;
  rotate_body_to_world(&state, ax, ay, az, &wx, &wy, &wz);
  const float g_ref = 1000.0f;
  wx -= 0.0f;
  wy -= 0.0f;
  wz -= g_ref;

  quat_to_euler(&state, &state.current.yaw, &state.current.pitch, &state.current.roll);
  state.current.x = wx;
  state.current.y = wy;
  state.current.z = wz;
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

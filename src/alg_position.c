#include "inc/alg_position.h"

#include <math.h>
#include <stdbool.h>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

typedef struct {
  float x;
  float y;
  float z;
  float vx;
  float vy;
  float vz;
  float bx;
  float by;
  float bz;
  float g_ref_mg;
  world_posture_t last;
  bool valid;
} pos_state_t;

static pos_state_t state = {
  // 使用 mg 作为加速度输入单位：1 g = 1000 mg
  .g_ref_mg = 1000.0f,
};

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

static inline float norm3(float x, float y, float z)
{
  return sqrtf(x * x + y * y + z * z);
}

static inline void rotate_body_to_world_euler(float bx, float by, float bz,
                                              float yaw_deg, float pitch_deg, float roll_deg,
                                              float *wx, float *wy, float *wz)
{
  const float deg2rad = (float)M_PI / 180.0f;
  float yaw = yaw_deg * deg2rad;
  float pitch = pitch_deg * deg2rad;
  float roll = roll_deg * deg2rad;

  float cy = cosf(yaw);
  float sy = sinf(yaw);
  float cp = cosf(pitch);
  float sp = sinf(pitch);
  float cr = cosf(roll);
  float sr = sinf(roll);

  float r11 = cy * cp;
  float r12 = cy * sp * sr - sy * cr;
  float r13 = cy * sp * cr + sy * sr;
  float r21 = sy * cp;
  float r22 = sy * sp * sr + cy * cr;
  float r23 = sy * sp * cr - cy * sr;
  float r31 = -sp;
  float r32 = cp * sr;
  float r33 = cp * cr;

  *wx = r11 * bx + r12 * by + r13 * bz;
  *wy = r21 * bx + r22 * by + r23 * bz;
  *wz = r31 * bx + r32 * by + r33 * bz;
}

world_position_t alg_position_update(const world_posture_t *posture,
                                     const imu_data_t *imu)
{
  if (!posture || !imu) {
    world_position_t empty = {
      .x = state.x,
      .y = state.y,
      .z = state.z,
      .yaw = state.last.yaw,
      .pitch = state.last.pitch,
      .roll = state.last.roll,
    };
    return empty;
  }

  world_posture_t posture_val = *posture;
  // 位置解算流程（基于原始加速度）：
  // 1) 使用姿态角（yaw/pitch/roll）将“原始 IMU 加速度”从机体坐标系旋转到世界坐标系
  // 2) 在世界坐标系去掉重力分量，得到线加速度
  // 3) 静止检测时估计偏置并清零速度，抑制积分漂移
  // 4) 积分得到速度与位置（只作为短时间相对位移参考）
  //
  // 重要：加速度输入单位为 mg（1 g = 1000 mg）。
  // 这里先在 mg 单位下做旋转和去重力，再统一换算到 m/s^2。

  world_position_t out = {
    .x = state.x,
    .y = state.y,
    .z = state.z,
    .yaw = posture_val.yaw,
    .pitch = posture_val.pitch,
    .roll = posture_val.roll,
  };

  if (!state.valid) {
    state.last = posture_val;
    state.valid = true;
    return out;
  }

  float dt = (float)ALG_POSITION_SAMPLE_US / 1000000.0f;
  if (!isfinite(dt) || dt <= 0.0f) {
    dt = 0.01f;
  }

  // 原始 IMU 加速度（mg）
  float ax_mg = imu->ax;
  float ay_mg = imu->ay;
  float az_mg = imu->az;

  if (!isfinite(ax_mg) || !isfinite(ay_mg) || !isfinite(az_mg)) {
    return out;
  }

  // 将机体系“原始加速度（mg）”旋转到世界坐标系
  float wx_mg, wy_mg, wz_mg;
  rotate_body_to_world_euler(ax_mg, ay_mg, az_mg,
                             posture_val.yaw, posture_val.pitch, posture_val.roll,
                             &wx_mg, &wy_mg, &wz_mg);

  // 世界系重力向量约为 (0, 0, +1g)；从原始加速度中去除
  float lin_wx_mg = wx_mg;
  float lin_wy_mg = wy_mg;
  float lin_wz_mg = wz_mg - state.g_ref_mg;

  // 统一换算到 m/s^2，避免后续阈值与积分混乱
  const float mg_to_mps2 = 0.00980665f;
  float lin_wx = lin_wx_mg * mg_to_mps2;
  float lin_wy = lin_wy_mg * mg_to_mps2;
  float lin_wz = lin_wz_mg * mg_to_mps2;

  float dy = angle_diff_deg(posture_val.yaw, state.last.yaw);
  float dp = angle_diff_deg(posture_val.pitch, state.last.pitch);
  float dr = angle_diff_deg(posture_val.roll, state.last.roll);
  float max_d = fmaxf(fmaxf(fabsf(dy), fabsf(dp)), fabsf(dr));
  float lin_norm = norm3(lin_wx, lin_wy, lin_wz);

  // 静止检测：角度变化小 + 线加速度小
  const float stationary_angle_deg = 1.0f;
  const float stationary_lin_mps2 = 0.2f;
  bool stationary = (max_d < stationary_angle_deg) && (lin_norm < stationary_lin_mps2);

  if (stationary) {
    // 静止时低通估计偏置，抑制积分漂移
    const float bias_alpha = 0.02f;
    state.bx = lpf_update(state.bx, lin_wx, bias_alpha);
    state.by = lpf_update(state.by, lin_wy, bias_alpha);
    state.bz = lpf_update(state.bz, lin_wz, bias_alpha);
    state.vx = 0.0f;
    state.vy = 0.0f;
    state.vz = 0.0f;

    // 静止时轻微修正 g 参考值（单位 mg），用于抵消量程误差
    float acc_norm_mg = norm3(ax_mg, ay_mg, az_mg);
    if (isfinite(acc_norm_mg)) {
      const float g_alpha = 0.01f;
      state.g_ref_mg = lpf_update(state.g_ref_mg, acc_norm_mg, g_alpha);
    }
  }

  // 去除静止偏置后再积分
  lin_wx -= state.bx;
  lin_wy -= state.by;
  lin_wz -= state.bz;

  // 速度积分
  state.vx += lin_wx * dt;
  state.vy += lin_wy * dt;
  state.vz += lin_wz * dt;

  // 位置积分
  state.x += state.vx * dt;
  state.y += state.vy * dt;
  state.z += state.vz * dt;

  state.last = posture_val;

  out.x = state.x;
  out.y = state.y;
  out.z = state.z;
  return out;
}

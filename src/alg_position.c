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
  float g_ref;
  world_posture_t last;
  bool valid;
} pos_state_t;

static pos_state_t state = {
  .g_ref = 9.80665f,
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

world_position_t alg_position_update(world_posture_t posture)
{
  // 位置解算流程：
  // 1) 将姿态输出的机体加速度旋转到世界坐标系
  // 2) 判断姿态输出的加速度是否包含重力分量（当前未知）
  // 3) 若包含重力，则在世界系 Z 轴方向去掉 g
  // 4) 静止检测时估计偏置并清零速度，抑制积分漂移
  // 5) 积分得到速度与位置（只作为短时间相对位移参考）
  //
  // 注意：姿态角已验证正确，线加速度是否含重力不确定。
  // 因此这里用幅值阈值做启发式判断，避免重复/遗漏重力补偿。

  world_position_t out = {
    .x = state.x,
    .y = state.y,
    .z = state.z,
    .yaw = posture.yaw,
    .pitch = posture.pitch,
    .roll = posture.roll,
  };

  if (!state.valid) {
    state.last = posture;
    state.valid = true;
    return out;
  }

  float dt = (float)ALG_POSITION_SAMPLE_US / 1000000.0f;
  if (!isfinite(dt) || dt <= 0.0f) {
    dt = 0.01f;
  }

  float ax = posture.ax;
  float ay = posture.ay;
  float az = posture.az;

  if (!isfinite(ax) || !isfinite(ay) || !isfinite(az)) {
    return out;
  }

  // 将机体系加速度旋转到世界坐标系
  float wx, wy, wz;
  rotate_body_to_world_euler(ax, ay, az, posture.yaw, posture.pitch, posture.roll, &wx, &wy, &wz);

  float acc_norm = norm3(ax, ay, az);
  // 判断姿态输出是否包含重力分量：
  // - 如果幅值接近 g，且并非极小值，则更可能是“原始加速度”（含重力）
  // - 否则认为是“线加速度”（已去重力）
  const float g_min_ratio = 0.7f;
  const float g_max_ratio = 1.3f;
  bool acc_includes_g = (acc_norm > state.g_ref * g_min_ratio) &&
                        (acc_norm < state.g_ref * g_max_ratio);

  // 线加速度（世界坐标）
  float lin_wx = wx;
  float lin_wy = wy;
  float lin_wz = wz;
  if (acc_includes_g) {
    // 世界系重力向量为 (0, 0, g)
    lin_wz -= state.g_ref;
  }

  float dy = angle_diff_deg(posture.yaw, state.last.yaw);
  float dp = angle_diff_deg(posture.pitch, state.last.pitch);
  float dr = angle_diff_deg(posture.roll, state.last.roll);
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

    // 静止且疑似包含重力时，轻微修正 g 参考值
    if (acc_includes_g && isfinite(acc_norm)) {
      const float g_alpha = 0.01f;
      state.g_ref = lpf_update(state.g_ref, acc_norm, g_alpha);
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

  state.last = posture;

  out.x = state.x;
  out.y = state.y;
  out.z = state.z;
  return out;
}

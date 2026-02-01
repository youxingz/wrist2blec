#pragma once
#include "_common.h"
#include <stdbool.h>
#include <stdint.h>

typedef struct {
  uint16_t sample_us;
  float balance_yaw;
  float balance_pitch;
  float balance_roll;
  float balance_yaw_thresh;
  float balance_pitch_thresh;
  float balance_roll_thresh;
} alg_config_t;

int alg_imu_init(alg_config_t config);

int alg_imu_update(imu_data_t * data);

int alg_imu_get_current(world_pos_t * position);

bool alg_imu_is_yaw_balanced(void);
bool alg_imu_is_pitch_balanced(void);
bool alg_imu_is_roll_balanced(void);

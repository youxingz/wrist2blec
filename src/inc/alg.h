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

#define ALG_AXIS_YAW   0x01
#define ALG_AXIS_PITCH 0x02
#define ALG_AXIS_ROLL  0x03

int alg_imu_init(alg_config_t config);

int alg_imu_update(imu_data_t * data);

int alg_imu_get_current(world_pos_t * position);

bool alg_imu_is_yaw_balanced(void);
bool alg_imu_is_pitch_balanced(void);
bool alg_imu_is_roll_balanced(void);

int alg_update_threshold(uint8_t axis, float threshold_deg);

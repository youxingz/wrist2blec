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
} alg_posture_config_t;

#define ALG_POSTURE_AXIS_YAW   0x01
#define ALG_POSTURE_AXIS_PITCH 0x02
#define ALG_POSTURE_AXIS_ROLL  0x03

int alg_posture_init(alg_posture_config_t config);

world_posture_t alg_posture_update(imu_data_t * data);

bool alg_posture_is_yaw_balanced(void);
bool alg_posture_is_pitch_balanced(void);
bool alg_posture_is_roll_balanced(void);

int alg_posture_update_threshold(uint8_t axis, float threshold_deg);

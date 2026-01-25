#pragma once
#include "_common.h"
#include <stdint.h>

typedef struct {
  uint16_t sample_us;
} alg_config_t;

int alg_imu_init(alg_config_t config);

int alg_imu_update(imu_data_t * data);

int alg_imu_get_current(world_pos_t * position);

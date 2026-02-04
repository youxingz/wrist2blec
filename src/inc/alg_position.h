#pragma once

#include "_common.h"

#ifndef ALG_POSITION_SAMPLE_US
#define ALG_POSITION_SAMPLE_US 10000
#endif

world_position_t alg_position_update(const world_posture_t *posture,
                                     const imu_data_t *imu);

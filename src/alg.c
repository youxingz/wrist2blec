#include "inc/alg.h"

static volatile alg_config_t config;

int alg_imu_init(alg_config_t cfg)
{
  config.sample_us = cfg.sample_us;
  return 0;
}

/**
 * 以固定频率采集并更新至此
 */
int alg_imu_update(imu_data_t * data)
{


  // if return 0, call: #alg_imu_get_current/1
  return 0;
}

/**
 * 获取解算出来的最新姿势（世界坐标）
 */
int alg_imu_get_current(world_pos_t * position)
{
  return 0;
}

#pragma once

typedef struct {
  float temperature;
  float ax;
  float ay;
  float az;
  float gx;
  float gy;
  float gz;
  float mx;
  float my;
  float mz;
} imu_data_t;

typedef struct {
  // float temperature;
  float ax;
  float ay;
  float az;
  float yaw;
  float pitch;
  float roll;
} world_pos_t;

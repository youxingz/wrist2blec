#pragma once

typedef struct {
  float temperature;
  float ax;  // Body Acc
  float ay;
  float az;
  float gx;  // Body Gyro
  float gy;
  float gz;
  float mx;  // Body Mag
  float my;
  float mz;
} imu_data_t;

typedef struct {
  // float temperature;
  float ax;     // World Acc
  float ay;
  float az;
  float yaw;    // World YPR
  float pitch;
  float roll;
} world_posture_t;

typedef struct {
  float x;
  float y;
  float z;      // always zero, ignore it.
  float yaw;
  float pitch;
  float roll;
} world_position_t;

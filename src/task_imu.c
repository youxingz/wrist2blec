#include "inc/task_imu.h"

#include "../drivers/icm_42688p.h"
#include "../drivers/lis3mdl.h"
#include "inc/alg_posture.h"
#include "inc/alg_position.h"
#include "inc/ble_aaaa.h"
#include "inc/pba.h"
#include "inc/storage.h"

#include <nrfx_timer.h>

#include <errno.h>
#include <zephyr/irq.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(task_imu, LOG_LEVEL_INF);

static int worker_init();

// nRF Timer
#define SAMPLE_TIME_MS 10  // timer: 2ms
#define TIMER_INST  NRF_TIMER1
static nrfx_timer_t timer = NRFX_TIMER_INSTANCE(TIMER_INST);
#define TASK_IMU_PRIORITY 3
// IMU
static volatile uint64_t dindex = 0;
static volatile bool dready = false;
static float range_accel;
static float range_gyro;
static float range_magnetic;

// static void task_imu_timer_interupt()
static void on_imu_timer_running(nrf_timer_event_t event_type, void * p_context)
{
  if(event_type == NRF_TIMER_EVENT_COMPARE0) {
    dindex++;
    dready = true;
  }
}

int task_imu_init()
{
  // timer
#if defined(__ZEPHYR__)
  IRQ_CONNECT(NRFX_IRQ_NUMBER_GET(TIMER_INST), IRQ_PRIO_LOWEST,
              nrfx_timer_irq_handler, &timer, 0);
#endif
  uint32_t base_frequency = NRF_TIMER_BASE_FREQUENCY_GET(timer.p_reg);
  nrfx_timer_config_t config = NRFX_TIMER_DEFAULT_CONFIG(base_frequency);
  // config.frequency = NRF_TIMER_FREQ_1MHz;
  config.bit_width = NRF_TIMER_BIT_WIDTH_32;
  config.interrupt_priority = TASK_IMU_PRIORITY;
  int err_code = nrfx_timer_init(&timer, &config, on_imu_timer_running);
  if (err_code == -EALREADY) {
    LOG_INF("IMU Timer init error: -EALREADY");
  } else if (err_code < 0) {
    LOG_INF("IMU Timer init error: %d", err_code);
  }

  // imu
  int err = icm_42688_init();
  if (err) {
    LOG_INF("ICM42688 init error: %d", err);
  }

  err = lis3dml_dev_init();
  if (err) {
    LOG_INF("LIS3DML init error: %d", err);
  }
  LOG_INF("[IMU] init successfully.");
  
  // alg
  alg_posture_config_t posture_config = {
    .sample_us = SAMPLE_TIME_MS * 1000, // 2ms
    .balance_yaw = 0.0f,
    .balance_pitch = 0.0f,
    .balance_roll = -90.0f,
    .balance_yaw_thresh = -1.0f,
    .balance_pitch_thresh = -1.0f,
    .balance_roll_thresh = 10.0f,
  };
  uint8_t thresh = 0;
  if (storage_read(ALG_POSTURE_AXIS_YAW, &thresh) == 0) {
    posture_config.balance_yaw_thresh = (float)thresh;
  }
  if (storage_read(ALG_POSTURE_AXIS_PITCH, &thresh) == 0) {
    posture_config.balance_pitch_thresh = (float)thresh;
  }
  if (storage_read(ALG_POSTURE_AXIS_ROLL, &thresh) == 0) {
    posture_config.balance_roll_thresh = (float)thresh;
  }

  err = alg_posture_init(posture_config);
  if (err) {
    LOG_INF("Alg init error: %d", err);
  }
  LOG_INF("[ALG] init successfully.");

  // worker
  worker_init();
  return 0;
}

int task_imu_uninit()
{
  lis3dml_uninit();
  // icm42688p: do nothing.
  return 0;
}

int task_imu_start()
{
  LOG_INF(">> [TASK/IMU] start.");
  // IMU: 500Hz
  int err = icm_42688_begin();
  if (err) {
    printk("icm_42688_begin() error: %d\n", err);
    return err;
  }
  // icm_42688_set_odr_fsr(/* who= */GYRO,/* ODR= */ODR_500HZ, /* FSR = */FSR_0);
  // icm_42688_set_odr_fsr(/* who= */ALL,/* ODR= */ODR_500HZ, /* FSR = */FSR_0);
  icm_42688_set_odr_fsr(/* who= */ALL,/* ODR= */ODR_8KHZ, /* FSR = */FSR_1);
  icm_42688_set_gyro_notch_filter_frequency(1000.0, ALL);
  icm_42688_set_gyro_notch_filter(true);
  icm_42688_set_aaf_bandwidth(ALL, 0); // ~1000Hz
  icm_42688_set_aaf(ALL, false);   // 抗锯齿
  icm_42688_start_temp_measure();
  icm_42688_start_gyro_measure(LN_MODE);
  icm_42688_start_accel_measure(LN_MODE);
  icm_42688_start_fifo_mode();
  
  // 磁力计: 40Hz
  lis3dml_reboot();
  k_msleep(10);
  lis3dml_soft_reset();
  k_msleep(10);
  lis3dml_init(false, LIS3DML_FREQUENCY_1000_HZ, LIS3DML_FULLSCALE_4_GAUSS);
  lis3dml_xy_power_mode(LIS3DML_ULTRA_HIGH_PERFORMANCE);
  lis3dml_z_power_mode(LIS3DML_ULTRA_HIGH_PERFORMANCE);
  lis3dml_operation_mode(LIS3DML_OPMODE_CONTINUOUS_CONVERSION);

  range_accel = icm_42688_get_range_accel();
  range_gyro = icm_42688_get_range_gyro();
  range_magnetic = lis3dml_get_range_magnetic();
  // LOG_INF("range_accel: %f, range_gyro: %f, range_magnetic: %f", range_accel, range_gyro, range_magnetic);

  dindex = 0;
  uint32_t desired_ticks = nrfx_timer_ms_to_ticks(&timer, SAMPLE_TIME_MS); // ms
  nrfx_timer_extended_compare(&timer, NRF_TIMER_CC_CHANNEL0, desired_ticks, NRF_TIMER_SHORT_COMPARE0_CLEAR_MASK, true);
  nrfx_timer_clear(&timer);
  nrfx_timer_enable(&timer);
  return 0;
}

int task_imu_stop()
{
  LOG_INF(">> [TASK/IMU] stop.");
  // k_timer_stop(&task_imu_timer);
  nrfx_timer_clear(&timer);
  if (nrfx_timer_is_enabled(&timer)) {
    nrfx_timer_disable(&timer);
  }
  dindex = 0;
  // low power -> icm42688
  return 0;
}


/*
 **************************************************
 *                                                *
 *  Worker Loop                                 *
 *                                                *
 **************************************************
 */


// Looper

#define LOOPER_INTERVAL         300     // 0.3ms
static void looper_work_handler(struct k_work *work);
static K_WORK_DELAYABLE_DEFINE(looper_work, looper_work_handler);
static int worker_init()
{
  int err = 0;
  // err = k_work_schedule(&looper_work, K_NO_WAIT);
  err = k_work_schedule(&looper_work, K_MSEC(200)); // 延迟 200ms 启动，确保尽可能在最后时刻
  return 0;
}

// helper func:
static int convert2buf(uint16_t index, imu_data_t * imu, world_position_t * position, uint8_t * buf)
{
  // ts:
  int i = 0;
  buf[i++] = (index >> 8) & 0xFF;
  buf[i++] = index & 0xFF;
  // imu:
  int16_t ax = (int16_t)(imu->ax / range_accel);
  int16_t ay = (int16_t)(imu->ay / range_accel);
  int16_t az = (int16_t)(imu->az / range_accel);
  int16_t gx = (int16_t)(imu->gx / range_gyro);
  int16_t gy = (int16_t)(imu->gy / range_gyro);
  int16_t gz = (int16_t)(imu->gz / range_gyro);
  int16_t mx = (int16_t)(imu->mx / range_magnetic);
  int16_t my = (int16_t)(imu->my / range_magnetic);
  int16_t mz = (int16_t)(imu->mz / range_magnetic);
  int16_t temperature = (int16_t)(imu->temperature * 100);
  buf[i++] = (ax >> 8) & 0xFF;
  buf[i++] = ax & 0xFF;
  buf[i++] = (ay >> 8) & 0xFF;
  buf[i++] = ay & 0xFF;
  buf[i++] = (az >> 8) & 0xFF;
  buf[i++] = az & 0xFF;
  buf[i++] = (gx >> 8) & 0xFF;
  buf[i++] = gx & 0xFF;
  buf[i++] = (gy >> 8) & 0xFF;
  buf[i++] = gy & 0xFF;
  buf[i++] = (gz >> 8) & 0xFF;
  buf[i++] = gz & 0xFF;
  buf[i++] = (mx >> 8) & 0xFF;
  buf[i++] = mx & 0xFF;
  buf[i++] = (my >> 8) & 0xFF;
  buf[i++] = my & 0xFF;
  buf[i++] = (mz >> 8) & 0xFF;
  buf[i++] = mz & 0xFF;
  buf[i++] = (temperature >> 8) & 0xFF;
  buf[i++] = temperature & 0xFF;
  // position:
  int16_t x = (int16_t)(position->x * 100);
  int16_t y = (int16_t)(position->y * 100);
  int16_t z = (int16_t)(position->z * 100);
  int16_t yaw = (int16_t)(position->yaw * 100);
  int16_t pitch = (int16_t)(position->pitch * 100);
  int16_t roll = (int16_t)(position->roll * 100);
  buf[i++] = (x >> 8) & 0xFF;
  buf[i++] = x & 0xFF;
  buf[i++] = (y >> 8) & 0xFF;
  buf[i++] = y & 0xFF;
  buf[i++] = (z >> 8) & 0xFF;
  buf[i++] = z & 0xFF;
  buf[i++] = (yaw >> 8) & 0xFF;
  buf[i++] = yaw & 0xFF;
  buf[i++] = (pitch >> 8) & 0xFF;
  buf[i++] = pitch & 0xFF;
  buf[i++] = (roll >> 8) & 0xFF;
  buf[i++] = roll & 0xFF;
  return i;
}


static void looper_work_handler(struct k_work *work)
{
  // SPI & ALG

  float magnetometer_tmp[3];
  imu_data_t current;
  world_posture_t posture;
  while(true) {
    if (!dready) {
      k_usleep(100);
      break;
    }
    dready = false;
    
    // fetch imux9 data.
    icm_42688_get_fifo_data();
    current.temperature = icm_42688_get_temperature();
    current.ax = icm_42688_get_accel_data_x();
    current.ay = icm_42688_get_accel_data_y();
    current.az = icm_42688_get_accel_data_z();
    current.gx = icm_42688_get_gyro_data_x();
    current.gy = icm_42688_get_gyro_data_y();
    current.gz = icm_42688_get_gyro_data_z();
    if (lis3dml_data_ready()) {
      if (LIS3DML_SUCCESS == lis3dml_xyz_magnitude(magnetometer_tmp)) {
        current.mx = magnetometer_tmp[0];
        current.my = magnetometer_tmp[1];
        current.mz = magnetometer_tmp[2];
      }
    }
    
    // Alg process:

    world_posture_t posture = alg_posture_update(&current);

    bool is_roll_balanced = alg_posture_is_roll_balanced();

    // 亮灯
    pba_motor_front_en(!is_roll_balanced);

    // pos:
    world_position_t position = alg_position_update(&posture, &current);

    // if BLE enabled, commit to GATT.
    // size: 2 + (1 + 9 + 6) * 2 = 34 bytes
    #define CM_BUFFER_SIZE 34
    uint8_t buf[CM_BUFFER_SIZE];
    int len = convert2buf(dindex, &current, &position, buf);
    ble_aaef_notify_commit(buf, len);
  }
	k_work_reschedule(k_work_delayable_from_work(work), K_USEC(LOOPER_INTERVAL));
}

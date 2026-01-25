/*!
 * @file icm_42688p.c
 * @brief Tokmas ICM-42688-PC compatible wrapper for the original DFRobot/TDK-style API.
 *
 * Notes:
 *  - This file keeps the public API unchanged (icm_42688_*), but remaps registers to Tokmas ICM-42688-PC.
 *  - Advanced features (FIFO/APEX/Tap/WOM/SMD/Notch/AAF/UI filter) are not implemented here (no-op or best-effort),
 *    because Tokmas datasheet exposes a different (and smaller) register map compared to the original banked map.
 *  - SPI/I2C transport is abstracted by icm_42688_{read,write}_reg() provided by your platform layer.
 */

#include "icm_42688p.h"

#ifdef TOKMAS_ICM42688P__
#include <math.h>
#include <zephyr/kernel.h>

#define delay(ms) k_msleep(ms)

/* ----------------------------- Tokmas register map -----------------------------
 * WHO_AM_I      : 0x00, expected 0x05
 * CTRL2 (Accel) : 0x03  [bit7 aST][6:4 aFS][3:0 aODR]
 * CTRL3 (Gyro)  : 0x04  [bit7 gST][6:4 gFS][3:0 gODR]
 * CTRL5 (LPF)   : 0x06  (optional)
 * CTRL7 (EN)    : 0x08  [7 SyncSample][5 DRDY_DIS][4 gSN][1 gEN][0 aEN]
 * TEMP_H/L      : 0x33/0x34
 * AX_H/L        : 0x35/0x36, AY_H/L 0x37/0x38, AZ_H/L 0x39/0x3A
 * GX_H/L        : 0x3B/0x3C, GY_H/L 0x3D/0x3E, GZ_H/L 0x3F/0x40
 * RESET         : 0x60  (write 0xB0)
 */

/* ----------------------------- module state ----------------------------- */
static uint8_t _mode;
static uint8_t _tapNum;
static uint8_t _tapAxis;
static uint8_t _tapDir;
static float _gyroRange;   /* dps per LSB */
static float _accelRange;  /* mg  per LSB */
static bool  FIFOMode;

static int16_t _accelX, _accelY, _accelZ;
static int16_t _gyroX, _gyroY, _gyroZ;
static int16_t _temp_raw;

/* CTRL2/CTRL3 cached values (Tokmas format) */
static uint8_t _ctrl2; /* accel */
static uint8_t _ctrl3; /* gyro  */
static uint8_t _ctrl7; /* enable */

/* ----------------------------- helpers ----------------------------- */

static inline int16_t s16_from_hl(uint8_t h, uint8_t l)
{
  return (int16_t)((uint16_t)h << 8 | (uint16_t)l);
}

/* Map original DFRobot/TDK ODR enum (1..15) to Tokmas ODR code (0..15).
 * We choose the closest practical setting.
 */
static uint8_t map_odr_tokmas(uint8_t odr)
{
  /* Tokmas ODR code table (from datasheet Table 23):
   * 0:7174.4, 1:3587.2, 2:1793.6, 3:896.8/1000, 4:448.4/500, 5:224.2/250,
   * 6:112.1/125, 7:56.05/62.5, 8:28.025/31.25, 12:128(LP),13:21(LP),14:11(LP),15:3(LP)
   * (Exact accel-only rates differ; in 6DOF the rate follows gyro nature frequency.)
   */
  switch (odr) {
    case ODR_32KHZ:   return 0;
    case ODR_16KHZ:   return 1;
    case ODR_8KHZ:    return 2;
    case ODR_4KHZ:    return 3;
    case ODR_2KHZ:    return 4;
    case ODR_1KHZ:    return 5;
    case ODR_500HZ:   return 6;
    case ODR_200HZ:   return 6;
    case ODR_100HZ:   return 7;
    case ODR_50HZ:    return 8;
    case ODR_25KHZ:   return 8;
    case ODR_12_5KHZ: return 8;
    case ODR_6_25KHZ: return 12;
    case ODR_3_125HZ: return 13;
    case ODR_1_5625HZ:return 14;
    default:          return 5; /* default ~224Hz/250Hz-ish */
  }
}

/* Map original accel FSR enum (0..3 => ±16/8/4/2 g) to Tokmas aFS code */
static uint8_t map_accel_fs_tokmas(uint8_t fsr)
{
  switch (fsr) {
    case FSR_0: return 3; /* ±16g */
    case FSR_1: return 2; /* ±8g  */
    case FSR_2: return 1; /* ±4g  */
    case FSR_3: return 0; /* ±2g  */
    default:    return 3;
  }
}

/* Map original gyro FSR enum (0..7 => ±2000..±15.625 dps) to Tokmas gFS code */
static uint8_t map_gyro_fs_tokmas(uint8_t fsr)
{
  switch (fsr) {
    case FSR_0: return 7; /* ±2048dps */
    case FSR_1: return 6; /* ±1024dps */
    case FSR_2: return 5; /* ±512dps  */
    case FSR_3: return 4; /* ±256dps  */
    case FSR_4: return 3; /* ±128dps  */
    case FSR_5: return 2; /* ±64dps   */
    case FSR_6: return 1; /* ±32dps   */
    case FSR_7: return 0; /* ±16dps   */
    default:    return 7;
  }
}

/* Compute scale factors from Tokmas FS codes */
static void update_scales_from_ctrl(void)
{
  uint8_t aFS = (_ctrl2 >> 4) & 0x07;
  uint8_t gFS = (_ctrl3 >> 4) & 0x07;

  /* accel FS code: 0:2g,1:4g,2:8g,3:16g */
  float a_g = 16.0f;
  switch (aFS) {
    case 0: a_g = 2.0f; break;
    case 1: a_g = 4.0f; break;
    case 2: a_g = 8.0f; break;
    case 3: a_g = 16.0f; break;
    default: a_g = 16.0f; break;
  }
  _accelRange = (2.0f * a_g * 1000.0f) / 65535.0f; /* mg/LSB */

  /* gyro FS code: 0:16,1:32,2:64,3:128,4:256,5:512,6:1024,7:2048 dps */
  float g_dps = 2048.0f;
  switch (gFS) {
    case 0: g_dps = 16.0f; break;
    case 1: g_dps = 32.0f; break;
    case 2: g_dps = 64.0f; break;
    case 3: g_dps = 128.0f; break;
    case 4: g_dps = 256.0f; break;
    case 5: g_dps = 512.0f; break;
    case 6: g_dps = 1024.0f; break;
    case 7: g_dps = 2048.0f; break;
    default: g_dps = 2048.0f; break;
  }
  _gyroRange = (2.0f * g_dps) / 65535.0f; /* dps/LSB */
}

static void write_ctrl2_ctrl3_ctrl7(void)
{
  icm_42688_write_reg(ICM42688_ACCEL_CONFIG0, &_ctrl2, 1); /* CTRL2 */
  icm_42688_write_reg(ICM42688_GYRO_CONFIG0,  &_ctrl3, 1); /* CTRL3 */
  icm_42688_write_reg(ICM42688_PWR_MGMT0,     &_ctrl7, 1); /* CTRL7 */
}

/* ----------------------------- public API ----------------------------- */

int icm_42688_init(void)
{
  /* Default behavior aligned with original driver defaults:
   *  - accel ODR ~= 1kHz, FS=±16g
   *  - gyro  ODR ~= 1kHz, FS=±2048dps
   *  - FIFO disabled
   */
  uint8_t aODR = map_odr_tokmas(ODR_1KHZ);
  uint8_t gODR = map_odr_tokmas(ODR_1KHZ);
  uint8_t aFS  = 3; /* 16g */
  uint8_t gFS  = 7; /* 2048dps */

  _ctrl2 = (0u << 7) | ((aFS & 0x7u) << 4) | (aODR & 0x0Fu);
  _ctrl3 = (0u << 7) | ((gFS & 0x7u) << 4) | (gODR & 0x0Fu);
  _ctrl7 = (0u << 7) | (0u << 5) | (0u << 4) | (1u << 1) | (1u << 0); /* enable gyro+accel, DRDY enabled */

  update_scales_from_ctrl();

  FIFOMode = false;
  _tapNum = 0;
  _tapAxis = 0;
  _tapDir = 0;

  return icm_42688_spi_init();
}

int icm_42688_begin(void)
{
  /* Verify WHO_AM_I first */
  uint8_t id = 0;
  if (icm_42688_read_reg(ICM42688_WHO_AM_I, &id, 1) == 0) {
    return ERR_DATA_BUS;
  }
  if (id != DFRobot_ICM42688_ID) {
    return ERR_IC_VERSION;
  }

  /* Soft reset: write 0xB0 to RESET register (0x60) */
  uint8_t rst = 0xB0;
  icm_42688_write_reg(ICM42688_DEVICE_CONFIG, &rst, 1);
  delay(10);

  /* Re-apply config and enable sensors */
  write_ctrl2_ctrl3_ctrl7();
  delay(2);

  return ERR_OK;
}

float icm_42688_get_temperature(void)
{
  /* Tokmas: T = raw / 256 (°C) per datasheet snippet.
   * (Some variants also define an offset; if you observe a fixed bias, apply it in the application.)
   */
  uint8_t data[2];
  icm_42688_read_reg(ICM42688_TEMP_DATA1, data, 2); /* TEMP_H then TEMP_L */
  _temp_raw = s16_from_hl(data[0], data[1]);
  return ((float)_temp_raw) / 256.0f;
}

float icm_42688_get_accel_data_x(void)
{
  uint8_t d[2];
  icm_42688_read_reg(ICM42688_ACCEL_DATA_X1, d, 2);
  _accelX = s16_from_hl(d[0], d[1]);
  return (float)_accelX * _accelRange;
}

float icm_42688_get_accel_data_y(void)
{
  uint8_t d[2];
  icm_42688_read_reg(ICM42688_ACCEL_DATA_Y1, d, 2);
  _accelY = s16_from_hl(d[0], d[1]);
  return (float)_accelY * _accelRange;
}

float icm_42688_get_accel_data_z(void)
{
  uint8_t d[2];
  icm_42688_read_reg(ICM42688_ACCEL_DATA_Z1, d, 2);
  _accelZ = s16_from_hl(d[0], d[1]);
  return (float)_accelZ * _accelRange;
}

float icm_42688_get_gyro_data_x(void)
{
  uint8_t d[2];
  icm_42688_read_reg(ICM42688_GYRO_DATA_X1, d, 2);
  _gyroX = s16_from_hl(d[0], d[1]);
  return (float)_gyroX * _gyroRange;
}

float icm_42688_get_gyro_data_y(void)
{
  uint8_t d[2];
  icm_42688_read_reg(ICM42688_GYRO_DATA_Y1, d, 2);
  _gyroY = s16_from_hl(d[0], d[1]);
  return (float)_gyroY * _gyroRange;
}

float icm_42688_get_gyro_data_z(void)
{
  uint8_t d[2];
  icm_42688_read_reg(ICM42688_GYRO_DATA_Z1, d, 2);
  _gyroZ = s16_from_hl(d[0], d[1]);
  return (float)_gyroZ * _gyroRange;
}

/* ---------------------- Optional / best-effort APIs ---------------------- */

void icm_42688_tap_detection_init(uint8_t accelMode) { (void)accelMode; _tapNum = 0; _tapAxis = 0; _tapDir = 0; }
void icm_42688_get_tap_information(void) {}
uint8_t icm_42688_number_of_tap(void) { return _tapNum; }
uint8_t icm_42688_axis_of_tap(void) { return _tapAxis; }

void icm_42688_wake_on_motion_init(void) {}
void icm_42688_set_wake_on_motion_threshold(uint8_t axis, uint8_t threshold) { (void)axis; (void)threshold; }
void icm_42688_set_wake_on_motion_interrupt(uint8_t axis) { (void)axis; }

void icm_42688_enable_smd_interrupt(uint8_t mode) { (void)mode; }

uint8_t icm_42688_read_interrupt_status(uint8_t reg)
{
  uint8_t status = 0;
  icm_42688_read_reg(reg, &status, 1);
  return status;
}

bool icm_42688_set_odr_fsr(uint8_t who, uint8_t ODR, uint8_t FSR)
{
  /* Remap caller's enums to Tokmas CTRL2/CTRL3 fields */
  if (who == ACCEL) {
    if (FSR > FSR_3) return false;
    uint8_t aODR = map_odr_tokmas(ODR);
    uint8_t aFS  = map_accel_fs_tokmas(FSR);
    _ctrl2 = (0u << 7) | ((aFS & 0x7u) << 4) | (aODR & 0x0Fu);
    icm_42688_write_reg(ICM42688_ACCEL_CONFIG0, &_ctrl2, 1);
  } else if (who == GYRO) {
    if (FSR > FSR_7) return false;
    uint8_t gODR = map_odr_tokmas(ODR);
    uint8_t gFS  = map_gyro_fs_tokmas(FSR);
    _ctrl3 = (0u << 7) | ((gFS & 0x7u) << 4) | (gODR & 0x0Fu);
    icm_42688_write_reg(ICM42688_GYRO_CONFIG0, &_ctrl3, 1);
  } else if (who == ALL) {
    /* Apply to both using same inputs */
    bool ok1 = icm_42688_set_odr_fsr(ACCEL, ODR, (FSR > FSR_3) ? FSR_0 : FSR);
    bool ok2 = icm_42688_set_odr_fsr(GYRO,  ODR, FSR);
    return ok1 && ok2;
  } else {
    return false;
  }

  update_scales_from_ctrl();
  return true;
}

/* FIFO not implemented for Tokmas map in this wrapper */
void icm_42688_start_fifo_mode(void) { FIFOMode = false; }
void icm_42688_sotp_fifo_mode(void)  { FIFOMode = false; }
void icm_42688_get_fifo_data(void)   {}

/* Interrupt pin mode config not implemented */
void icm_42688_set_int_mode(uint8_t INTPin, uint8_t INTmode, uint8_t INTPolarity, uint8_t INTDriveCircuit)
{
  (void)INTPin; (void)INTmode; (void)INTPolarity; (void)INTDriveCircuit;
}

/* Start/stop measurements: best-effort enable bits in CTRL7 */
void icm_42688_start_gyro_measure(uint8_t mode)
{
  (void)mode;
  _ctrl7 |= (1u << 1); /* gEN */
  icm_42688_write_reg(ICM42688_PWR_MGMT0, &_ctrl7, 1);
}

void icm_42688_start_accel_measure(uint8_t mode)
{
  (void)mode;
  _ctrl7 |= (1u << 0); /* aEN */
  icm_42688_write_reg(ICM42688_PWR_MGMT0, &_ctrl7, 1);
}

void icm_42688_start_temp_measure(void)
{
  /* Temperature is read-only output; nothing to enable in Tokmas basic map */
}

void icm_42688_set_fifo_data_mode(void) {}

void icm_42688_set_gyro_notch_filter_frequency(double freq, uint8_t axis) { (void)freq; (void)axis; }
void icm_42688_set_gyro_notch_filter_bandwidth(uint8_t bw) { (void)bw; }
void icm_42688_set_gyro_notch_filter(bool mode) { (void)mode; }

void icm_42688_set_aaf_bandwidth(uint8_t who, uint8_t BWIndex) { (void)who; (void)BWIndex; }
void icm_42688_set_aaf(uint8_t who, bool mode) { (void)who; (void)mode; }

bool icm_42688_set_ui_filter(uint8_t who, uint8_t filterOrder, uint8_t UIFilterIndex)
{
  (void)who; (void)filterOrder; (void)UIFilterIndex;
  return false;
}

float icm_42688_get_range_accel(void) { return _accelRange; }
float icm_42688_get_range_gyro(void)  { return _gyroRange; }

#endif

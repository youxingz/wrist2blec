#ifndef __LIS3MDL__H
#define __LIS3MDL__H

#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef enum {
  LIS3DML_SUCCESS = 0,
  LIS3DML_ERROR,
  LIS3DML_NOT_ENABLED,
  LIS3DML_NOT_INITIALIZED,
} lis3dml_err_t;

typedef enum {
  LIS3DML_LOWPOWER,
  LIS3DML_MEDIUM_PERFORMANCE,
  LIS3DML_HIGH_PERFORMANCE,
  LIS3DML_ULTRA_HIGH_PERFORMANCE,
} lis3dml_power_mode_t;

typedef enum {
  LIS3DML_FREQUENCY_0_625_HZ,
  LIS3DML_FREQUENCY_1_25_HZ,
  LIS3DML_FREQUENCY_2_5_HZ,
  LIS3DML_FREQUENCY_5_HZ,
  LIS3DML_FREQUENCY_10_HZ,
  LIS3DML_FREQUENCY_20_HZ,
  LIS3DML_FREQUENCY_40_HZ,
  LIS3DML_FREQUENCY_80_HZ,
  LIS3DML_FREQUENCY_155_HZ,
  LIS3DML_FREQUENCY_300_HZ,
  LIS3DML_FREQUENCY_560_HZ,
  LIS3DML_FREQUENCY_1000_HZ,
} lis3dml_sample_frequency_t;

typedef enum {
  LIS3DML_FULLSCALE_4_GAUSS,
  LIS3DML_FULLSCALE_8_GAUSS,
  LIS3DML_FULLSCALE_12_GAUSS,
  LIS3DML_FULLSCALE_16_GAUSS,
} lis3dml_fullscale_t;

typedef enum {
  LIS3DML_OPMODE_CONTINUOUS_CONVERSION,
  LIS3DML_OPMODE_SIGNLE_CONVERSION,     // only work for 0.625Hz - 80Hz.
  LIS3DML_OPMODE_POWER_DOWN,
} lis3dml_operation_mode_t;


/**
 * @brief init lis3dml module
 * @param enable_temperature enable temperature sensor
 * @param odr                sample frequency (0.625Hz - 1000Hz)
 * @param fullscale          full scale (4gauss - 16gauss)
*/
lis3dml_err_t lis3dml_init(bool enable_temperature, lis3dml_sample_frequency_t odr, lis3dml_fullscale_t fullscale);

/**
 * @brief uninit lis3dml module
*/
lis3dml_err_t lis3dml_uninit();

/**
 * @brief read chip id (must be 0x3D)
*/
uint8_t lis3dml_chip_id();

/**
 * @brief reboot chip
*/
lis3dml_err_t lis3dml_reboot();

/**
 * @brief soft reset
*/
lis3dml_err_t lis3dml_soft_reset();

/**
 * @brief set operation mode
 * @param mode op mode
*/
lis3dml_err_t lis3dml_operation_mode(lis3dml_operation_mode_t mode);

/**
 * @brief set x-axis and y-axis power mode
 * @param mode power mode
*/
lis3dml_err_t lis3dml_xy_power_mode(lis3dml_power_mode_t mode);

/**
 * @brief set z-axis power mode
 * @param mode power mode
*/
lis3dml_err_t lis3dml_z_power_mode(lis3dml_power_mode_t mode);

/**
 * @brief status bits
 * ZYXOR ZOR YOR XOR ZYXDA ZDA YDA XDA
*/
uint8_t lis3dml_status();

/**
 * @brief SPI data is ready?
 */
bool lis3dml_data_ready();

/**
 * @brief read xyz magnitude
 * @param data array size = 3
*/
lis3dml_err_t lis3dml_xyz_magnitude(float * data);

/**
 * @brief read temperature in ˚C
 * @param data temperature pointer
*/
lis3dml_err_t lis3dml_temperature(float * data);

/**
  * @}
  */

/**  implement by spi/i2c  */

lis3dml_err_t lis3dml_dev_init();

lis3dml_err_t lis3dml_read_registers(uint8_t address, uint8_t * data, uint8_t len);

lis3dml_err_t lis3dml_write_registers(uint8_t address, uint8_t * data, uint8_t len);

uint8_t lis3dml_read_register(uint8_t address);

lis3dml_err_t lis3dml_write_register(uint8_t address, uint8_t data);

#ifdef __cplusplus
}
#endif

#endif /* __LIS3MDL__H */


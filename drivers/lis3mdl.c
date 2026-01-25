#include "lis3mdl.h"

/************** Chip ID  *******************/

#define LIS3MDL_CHIPID              ((uint8_t)0x3D)

/************** Device Register  *******************/

#define LIS3MDL_MAG_WHO_AM_I_REG    0x0F
#define LIS3MDL_MAG_CTRL_REG1       0x20
#define LIS3MDL_MAG_CTRL_REG2       0x21
#define LIS3MDL_MAG_CTRL_REG3       0x22
#define LIS3MDL_MAG_CTRL_REG4       0x23
#define LIS3MDL_MAG_CTRL_REG5       0x24
#define LIS3MDL_MAG_STATUS_REG      0x27
#define LIS3MDL_MAG_OUTX_L          0x28
#define LIS3MDL_MAG_OUTX_H          0x29
#define LIS3MDL_MAG_OUTY_L          0x2A
#define LIS3MDL_MAG_OUTY_H          0x2B
#define LIS3MDL_MAG_OUTZ_L          0x2C
#define LIS3MDL_MAG_OUTZ_H          0x2D
#define LIS3MDL_MAG_TEMP_OUT_L      0x2E
#define LIS3MDL_MAG_TEMP_OUT_H      0x2F
#define LIS3MDL_MAG_INT_CFG         0x30
#define LIS3MDL_MAG_INT_SRC         0x31
#define LIS3MDL_MAG_INT_THS_L       0x32
#define LIS3MDL_MAG_INT_THS_H       0x33
  
/* Mag Temperature Sensor Control*/ 
#define LIS3MDL_MAG_TEMPSENSOR_ENABLE        ((uint8_t) 0x80)   /*!< Temp sensor Enable */
#define LIS3MDL_MAG_TEMPSENSOR_DISABLE       ((uint8_t) 0x00)   /*!< Temp sensor Disable */

/* Mag_XY-axis Operating Mode */ 
#define LIS3MDL_MAG_OM_XY_LOWPOWER           ((uint8_t) 0x00)
#define LIS3MDL_MAG_OM_XY_MEDIUM             ((uint8_t) 0x20)
#define LIS3MDL_MAG_OM_XY_HIGH               ((uint8_t) 0x40)
#define LIS3MDL_MAG_OM_XY_ULTRAHIGH          ((uint8_t) 0x60)
   
/* Mag Data Rate */ 
#define LIS3MDL_MAG_ODR_0_625_HZ             ((uint8_t) 0x00)  /*!< Output Data Rate = 0.625 Hz */
#define LIS3MDL_MAG_ODR_1_25_HZ              ((uint8_t) 0x04)  /*!< Output Data Rate = 1.25 Hz  */
#define LIS3MDL_MAG_ODR_2_5_HZ               ((uint8_t) 0x08)  /*!< Output Data Rate = 2.5 Hz   */
#define LIS3MDL_MAG_ODR_5_0_HZ               ((uint8_t) 0x0C)  /*!< Output Data Rate = 5.0 Hz   */
#define LIS3MDL_MAG_ODR_10_HZ                ((uint8_t) 0x10)  /*!< Output Data Rate = 10 Hz    */
#define LIS3MDL_MAG_ODR_20_HZ                ((uint8_t) 0x14)  /*!< Output Data Rate = 20 Hz    */
#define LIS3MDL_MAG_ODR_40_HZ                ((uint8_t) 0x18)  /*!< Output Data Rate = 40 Hz    */
#define LIS3MDL_MAG_ODR_80_HZ                ((uint8_t) 0x1C)  /*!< Output Data Rate = 80 Hz    */

/* Mag Data Rate */ 
#define LMS303C_MAG_SELFTEST_DISABLE         ((uint8_t 0x00)
#define LMS303C_MAG_SELFTEST_ENABLE          ((uint8_t 0x01)
   
/* Mag Full Scale */ 
#define LIS3MDL_MAG_FS_DEFAULT               ((uint8_t) 0x00)
#define LIS3MDL_MAG_FS_4_GA                  ((uint8_t) 0x00)  
#define LIS3MDL_MAG_FS_8_GA                  ((uint8_t) 0x20)
#define LIS3MDL_MAG_FS_12_GA                 ((uint8_t) 0x40)  
#define LIS3MDL_MAG_FS_16_GA                 ((uint8_t) 0x60)  /*!< Full scale = ±16 Gauss */

/* Mag_Reboot */ 
#define LIS3MDL_MAG_REBOOT_DEFAULT           ((uint8_t) 0x00)
#define LIS3MDL_MAG_REBOOT_ENABLE            ((uint8_t) 0x08)
   
/* Mag Soft reset */ 
#define LIS3MDL_MAG_SOFT_RESET_DEFAULT       ((uint8_t) 0x00)
#define LIS3MDL_MAG_SOFT_RESET_ENABLE        ((uint8_t) 0x04)
   
/* Mag_Communication_Mode */ 
#define LIS3MDL_MAG_SIM_4_WIRE               ((uint8_t) 0x00)
#define LIS3MDL_MAG_SIM_3_WIRE               ((uint8_t) 0x04)
   
/* Mag Lowpower mode config */ 
#define LIS3MDL_MAG_CONFIG_NORMAL_MODE       ((uint8_t) 0x00)
#define LIS3MDL_MAG_CONFIG_LOWPOWER_MODE     ((uint8_t) 0x20)
   
/* Mag Operation Mode */ 
#define LIS3MDL_MAG_SELECTION_MODE           ((uint8_t) 0x03) /* CTRL_REG3 */
#define LIS3MDL_MAG_CONTINUOUS_MODE          ((uint8_t) 0x00)
#define LIS3MDL_MAG_SINGLE_MODE              ((uint8_t) 0x01)
#define LIS3MDL_MAG_POWERDOWN1_MODE          ((uint8_t) 0x02)
#define LIS3MDL_MAG_POWERDOWN2_MODE          ((uint8_t) 0x03)

/* Mag_Z-axis Operation Mode */ 
#define LIS3MDL_MAG_OM_Z_LOWPOWER            ((uint8_t) 0x00)
#define LIS3MDL_MAG_OM_Z_MEDIUM              ((uint8_t) 0x04)
#define LIS3MDL_MAG_OM_Z_HIGH                ((uint8_t) 0x08)
#define LIS3MDL_MAG_OM_Z_ULTRAHIGH           ((uint8_t) 0x0C)   

/* Mag Big little-endian selection */ 
#define LIS3MDL_MAG_BLE_LSB                  ((uint8_t) 0x00)
#define LIS3MDL_MAG_BLE_MSB                  ((uint8_t) 0x02)


/* Mag_Bloc_update_magnetic_data */ 
#define LIS3MDL_MAG_BDU_CONTINUOUS           ((uint8_t) 0x00)
#define LIS3MDL_MAG_BDU_MSBLSB               ((uint8_t) 0x40)
   
   
/* Magnetometer_Sensitivity */
#define LIS3MDL_MAG_SENSITIVITY_FOR_FS_4GA   ((float)0.14f)  /**< Sensitivity value for 4 gauss full scale  [mgauss/LSB] */
#define LIS3MDL_MAG_SENSITIVITY_FOR_FS_8GA   ((float)0.29f)  /**< Sensitivity value for 8 gauss full scale  [mgauss/LSB] */
#define LIS3MDL_MAG_SENSITIVITY_FOR_FS_12GA  ((float)0.43f)  /**< Sensitivity value for 12 gauss full scale [mgauss/LSB] */
#define LIS3MDL_MAG_SENSITIVITY_FOR_FS_16GA  ((float)0.58f)  /**< Sensitivity value for 16 gauss full scale [mgauss/LSB] */



// Implementation

static float sensitivity = LIS3MDL_MAG_SENSITIVITY_FOR_FS_4GA;

/**
 * @brief init lis3dml module
 * @param enable_temperature enable temperature sensor
 * @param odr                sample frequency (0.625Hz - 1000Hz)
 * @param fullscale          full scale (4gauss - 16gauss)
*/
lis3dml_err_t lis3dml_init(bool enable_temperature, lis3dml_sample_frequency_t odr, lis3dml_fullscale_t fullscale)
{
  uint8_t ctrl1 = 0x00;
  uint8_t ctrl2 = 0x00;
  uint8_t ctrl3 = 0x00;

  ctrl1 |= (enable_temperature ? 1 : 0) << 7;
  if (odr <= LIS3DML_FREQUENCY_80_HZ) {
    ctrl1 |= (odr) << 2;
    ctrl3 = LIS3DML_OPMODE_SIGNLE_CONVERSION;
  } else {
    ctrl1 |= 1 << 1; // enable fast odr.
    ctrl1 |= (odr - LIS3DML_FREQUENCY_155_HZ) << 5; // equal to x-y operation mode.
    ctrl3 = LIS3DML_OPMODE_CONTINUOUS_CONVERSION;
  }
  
  switch(fullscale) {
    case LIS3DML_FULLSCALE_4_GAUSS: { sensitivity = LIS3MDL_MAG_SENSITIVITY_FOR_FS_4GA; break; }
    case LIS3DML_FULLSCALE_8_GAUSS: { sensitivity = LIS3MDL_MAG_SENSITIVITY_FOR_FS_8GA; break; }
    case LIS3DML_FULLSCALE_12_GAUSS: { sensitivity = LIS3MDL_MAG_SENSITIVITY_FOR_FS_12GA; break; }
    case LIS3DML_FULLSCALE_16_GAUSS: { sensitivity = LIS3MDL_MAG_SENSITIVITY_FOR_FS_16GA; break; }
  }
  ctrl2 |= fullscale << 5;

  lis3dml_err_t err = LIS3DML_SUCCESS;
  err = lis3dml_write_register(LIS3MDL_MAG_CTRL_REG1, ctrl1);
  if (err) return err;
  err = lis3dml_write_register(LIS3MDL_MAG_CTRL_REG2, ctrl2);
  if (err) return err;
  err = lis3dml_write_register(LIS3MDL_MAG_CTRL_REG3, ctrl3);
  if (err) return err;

  return LIS3DML_SUCCESS;
}

/**
 * @brief uninit lis3dml module
*/
lis3dml_err_t lis3dml_uninit()
{
  uint8_t ctrl = 0x00;
  /* Read control register 1 value */
  ctrl = lis3dml_read_register(LIS3MDL_MAG_CTRL_REG3);

  /* Clear Selection Mode bits */
  ctrl &= ~(LIS3MDL_MAG_SELECTION_MODE);

  /* Set Power down */
  ctrl |= LIS3MDL_MAG_POWERDOWN2_MODE;

  /* write back control register */
  return lis3dml_write_register(LIS3MDL_MAG_CTRL_REG3, ctrl);
}

/**
 * @brief read chip id (must be 0x3D)
*/
uint8_t lis3dml_chip_id()
{
  return lis3dml_read_register(LIS3MDL_MAG_WHO_AM_I_REG);
}

/**
 * @brief reboot chip
*/
lis3dml_err_t lis3dml_reboot()
{
  uint8_t ctrl = 0x01 << 3;
  return lis3dml_write_register(LIS3MDL_MAG_CTRL_REG2, ctrl);
}

/**
 * @brief soft reset
*/
lis3dml_err_t lis3dml_soft_reset()
{
  uint8_t ctrl = 0x01 << 2;
  return lis3dml_write_register(LIS3MDL_MAG_CTRL_REG2, ctrl);
}

/**
 * @brief set operation mode
 * @param mode op mode
*/
lis3dml_err_t lis3dml_operation_mode(lis3dml_operation_mode_t mode)
{
  uint8_t ctrl = mode;
  return lis3dml_write_register(LIS3MDL_MAG_CTRL_REG3, ctrl);
}

/**
 * @brief set x-axis and y-axis power mode
 * @param mode power mode
*/
lis3dml_err_t lis3dml_xy_power_mode(lis3dml_power_mode_t mode)
{
  uint8_t ctrl = 0x00;
  ctrl = lis3dml_read_register(LIS3MDL_MAG_CTRL_REG1);
  uint8_t bitmask = 0b0011111; // remain other bits.
  ctrl &= bitmask;
  ctrl |= mode << 5;
  return lis3dml_write_register(LIS3MDL_MAG_CTRL_REG1, ctrl);
}

/**
 * @brief set z-axis power mode
 * @param mode power mode
*/
lis3dml_err_t lis3dml_z_power_mode(lis3dml_power_mode_t mode)
{
  uint8_t ctrl = mode << 2;
  return lis3dml_write_register(LIS3MDL_MAG_CTRL_REG4, ctrl);
}

/**
 * @brief status bits
 * ZYXOR ZOR YOR XOR ZYXDA ZDA YDA XDA
*/
uint8_t lis3dml_status()
{
  return lis3dml_read_register(LIS3MDL_MAG_STATUS_REG);
}

/**
 * @brief read xyz magnitude
 * @param data array size = 3
*/
lis3dml_err_t lis3dml_xyz_magnitude(float * value)
{
  uint8_t data[6];
  lis3dml_err_t err = lis3dml_read_registers(LIS3MDL_MAG_OUTX_L, data, 6);
  if (err) return err;

  uint16_t sample_value = 0;
  for (int i = 0; i < 3; i++) {
    sample_value = ((((uint16_t)data[2 * i + 1]) << 8) | (uint16_t)(data[2 * i]));
    value[i] = -((int16_t)sample_value) * sensitivity;
  }

  return LIS3DML_SUCCESS;
}

/**
 * @brief read temperature in ˚C
 * @param data temperature pointer
*/
lis3dml_err_t lis3dml_temperature(float * value)
{
  uint8_t data[2];
  lis3dml_err_t err = lis3dml_read_registers(LIS3MDL_MAG_TEMP_OUT_L, data, 2);
  if (err) return err;

  uint16_t sample_value = (((uint16_t)data[1] << 8) | (uint16_t)(data[0]));
  *value = sample_value;

  return LIS3DML_SUCCESS;
}

float lis3dml_get_range_magnetic()
{
  return sensitivity;
}

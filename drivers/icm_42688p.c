/*!
 * @file DFRobot_ICM42688.cpp
 * @brief Define basic structure of DFRobot_ICM42688 class, the implementation of basic method
 * @copyright	Copyright (c) 2010 DFRobot Co.Ltd (http://www.dfrobot.com)
 * @license     The MIT License (MIT)
 * @author [yangfeng]<feng.yang@dfrobot.com>
 * @version V1.0
 * @date 2021-05-13
 * @url  https://github.com/DFRobot/DFRobot_ICM42688
 */
#include "icm_42688p.h"
#include "math.h"
#include <zephyr/kernel.h>

#define delay(ms) k_msleep(ms)

// private:
uint8_t _r, _g, _b;
uint8_t _mode;
uint8_t _tapNum;
uint8_t _tapAxis;
uint8_t _tapDir;
float _gyroRange;
float _accelRange;
bool FIFOMode;
int16_t _accelX;
int16_t _accelY;
int16_t _accelZ;
int16_t _gyroX;
int16_t _gyroZ;
int16_t _gyroY;
int8_t _temp;
int8_t _INTPin;

icm_42688_accel_config0_t accelConfig0;
icm_42688_pwrmgmt0_t PWRMgmt0;
icm_42688_intf_config1_t INTFConfig1;
icm_42688_accel_config1_t accelConfig1;
icm_42688_gyro_accel_config0_t gyroAccelConfig0;
icm_42688_apex_config7_t APEXConfig7;
icm_42688_apex_config8_t APEXConfig8;
icm_42688_smd_config_t SMDConfig;
icm_42688_gyro_config1_t  gyroConfig1;
icm_42688_fifo_config1_t FIFOConfig1;
icm_42688_int_config_t INTConfig;
icm_42688_gyro_config0_t gyroConfig0;
icm_42688_apex_config0_t APEXConfig0;
icm_42688_gyro_config_static9_t gyroConfigStatic9;
icm_42688_gyro_config_static2_t gyroConfigStatic2;
icm_42688_gyro_config_static5_t gyroConfigStatic5;
icm_42688_accel_config_static2_t accelConfigStatic2;
icm_42688_accel_config_static4_t accelConfigStatic4;
icm_42688_int_source_t  INTSource;


int icm_42688_init()
{
  accelConfig0.accelODR = 6;
  accelConfig0.accelFsSel = 0;
  gyroConfig0.gyroODR = 6;
  gyroConfig0.gyroFsSel = 0;
  _gyroRange = 4000/65535.0;
  _accelRange = 0.488f;
  FIFOMode = false;
  return icm_42688_spi_init();
}

int icm_42688_begin(void)
{
  uint8_t bank = 0;
  icm_42688_write_reg(ICM42688_REG_BANK_SEL, &bank, 1);
  uint8_t id=0;
  if(icm_42688_read_reg(ICM42688_WHO_AM_I, &id, 1) == 0){
    //DBG("bus data access error");
    return ERR_DATA_BUS;
  }
  DBG("real sensor id=");DBG(id);
  if(id != DFRobot_ICM42688_ID){
    return ERR_IC_VERSION;
  }
  uint8_t reset = 0;
  icm_42688_write_reg(ICM42688_DEVICE_CONFIG, &reset, 1);
  delay(2);
  return ERR_OK;
}

float icm_42688_get_temperature(void)
{
  float value;
  if(FIFOMode){
    value = (_temp/2.07) + 25;
  } else{
    uint8_t data[2];
    int16_t value2;
    icm_42688_read_reg(ICM42688_TEMP_DATA1, data, 2);
    value2 = ((uint16_t)data[0]<<8) | (uint16_t )data[1];
    value = value2/132.48 + 25;
  }
  return value;
}

float icm_42688_get_accel_data_x(void)
{
  float value;
  if(FIFOMode){
    value = _accelX;
  } else{
    uint8_t data[2];
    icm_42688_read_reg(ICM42688_ACCEL_DATA_X1, data, 2);
    int16_t value1 = ((uint16_t )data[0] << 8) | (uint16_t)data[1];
    value = value1;
  }
  return value*_accelRange;
}

float icm_42688_get_accel_data_y(void)
{
  float value;
  if(FIFOMode){
    value = _accelY;
  } else{
    uint8_t data[2];
    icm_42688_read_reg(ICM42688_ACCEL_DATA_Y1, data, 2);
    int16_t value1 = ((uint16_t )data[0] << 8) | (uint16_t)data[1];
    value = value1;
  }
  return value*_accelRange;
}

float icm_42688_get_accel_data_z(void)
{
  float value;
  if(FIFOMode){
    value = _accelZ;
  } else{
    uint8_t data[2];
    icm_42688_read_reg(ICM42688_ACCEL_DATA_Z1, data, 2);
    int16_t value1 = ((uint16_t )data[0] << 8) | (uint16_t)data[1];
    value = value1;
  }
  return value*_accelRange;
}

float icm_42688_get_gyro_data_x(void)
{
  float value;
  if(FIFOMode){
    value = _gyroX;
  } else{
    uint8_t data[2];
    icm_42688_read_reg(ICM42688_GYRO_DATA_X1, data, 2);
    int16_t value1 = ((uint16_t )data[0] << 8) | (uint16_t)data[1];
    value = value1;
  }
  return value*_gyroRange;
}

float icm_42688_get_gyro_data_y(void)
{
  float value;
  if(FIFOMode){
    value = _gyroY;
  } else{
    uint8_t data[2];
    icm_42688_read_reg(ICM42688_GYRO_DATA_Y1, data, 2);
    int16_t value1 = ((uint16_t )data[0] << 8) | (uint16_t)data[1];
    value = value1;
  }
  return value*_gyroRange;
}

float icm_42688_get_gyro_data_z(void)
{
  float value;
  if(FIFOMode){
    value = _gyroZ;
  } else{
    uint8_t data[2];
    icm_42688_read_reg(ICM42688_GYRO_DATA_Z1, data, 2);
    int16_t value1 = ((uint16_t )data[0] << 8) | (uint16_t)data[1];
    value = value1;
  }
  return value*_gyroRange;
}

void icm_42688_tap_detection_init(uint8_t accelMode)
{
  uint8_t bank = 0;
  icm_42688_write_reg(ICM42688_REG_BANK_SEL, &bank, 1);
  if(accelMode == 0){
    accelConfig0.accelODR = 15;
    icm_42688_write_reg(ICM42688_ACCEL_CONFIG0, &accelConfig0, 1);
    PWRMgmt0.accelMode = 2;
    icm_42688_write_reg(ICM42688_PWR_MGMT0, &PWRMgmt0, 1);
    delay(1);
    INTFConfig1.accelLpClkSel = 0;
    icm_42688_write_reg(ICM42688_INTF_CONFIG1, &INTFConfig1, 1);
    accelConfig1.accelUIFiltORD = 2;
    icm_42688_write_reg(ICM42688_ACCEL_CONFIG1, &accelConfig1, 1);
    gyroAccelConfig0.accelUIFiltBW = 0;
    icm_42688_write_reg(ICM42688_GYRO_ACCEL_CONFIG0, &gyroAccelConfig0, 1);
  } else if(accelMode == 1){
    accelConfig0.accelODR = 6;
    icm_42688_write_reg(ICM42688_ACCEL_CONFIG0, &accelConfig0, 1);
    PWRMgmt0.accelMode = 3;
    icm_42688_write_reg(ICM42688_PWR_MGMT0, &PWRMgmt0, 1);
    delay(1);
    accelConfig1.accelUIFiltORD = 2;
    icm_42688_write_reg(ICM42688_ACCEL_CONFIG1, &accelConfig1, 1);
    gyroAccelConfig0.accelUIFiltBW = 0;
    icm_42688_write_reg(ICM42688_GYRO_ACCEL_CONFIG0, &gyroAccelConfig0, 1);
  } else{
    DBG("accelMode invalid !");
    return;
  }
  delay(1);
  bank = 4;
  icm_42688_write_reg(ICM42688_REG_BANK_SEL, &bank, 1);
  APEXConfig8.tapTmin = 3;
  APEXConfig8.tapTavg = 3;
  APEXConfig8.tapTmax = 2;
  icm_42688_write_reg(ICM42688_APEX_CONFIG8, &APEXConfig8, 1);
  APEXConfig7.tapMinJerkThr = 17;
  APEXConfig7.tapMaxPeakTol = 1;
  icm_42688_write_reg(ICM42688_APEX_CONFIG7, &APEXConfig7, 1);
  delay(1);
  INTSource.tapDetIntEn = 1;
  if(_INTPin==1){
    icm_42688_write_reg(ICM42688_INT_SOURCE6, &INTSource, 1);
  } else {
    icm_42688_write_reg(ICM42688_INT_SOURCE7, &INTSource, 1);
  }
  delay(50);
  bank = 0;
  icm_42688_write_reg(ICM42688_REG_BANK_SEL, &bank, 1);
  APEXConfig0.tapEnable = 1;
  icm_42688_write_reg(ICM42688_APEX_CONFIG0, &APEXConfig0, 1);
}
void icm_42688_get_tap_information()
{
  uint8_t data;
  icm_42688_read_reg(ICM42688_APEX_DATA4, &data, 1);
  _tapNum = data & 0x18;
  _tapAxis = data & 0x06;
  _tapDir = data & 0x01;
}
uint8_t icm_42688_number_of_tap()
{
  return _tapNum;
}
uint8_t icm_42688_axis_of_tap()
{
  return _tapAxis;
}
void icm_42688_wake_on_motion_init()
{
  uint8_t bank = 0;
  icm_42688_write_reg(ICM42688_REG_BANK_SEL, &bank, 1);
  accelConfig0.accelODR = 9;
  icm_42688_write_reg(ICM42688_ACCEL_CONFIG0, &accelConfig0, 1);
  PWRMgmt0.accelMode = 2;
  icm_42688_write_reg(ICM42688_PWR_MGMT0, &PWRMgmt0, 1);
  delay(1);
  INTFConfig1.accelLpClkSel = 0;
  icm_42688_write_reg(ICM42688_INTF_CONFIG1, &INTFConfig1, 1);
  delay(1);
}
void icm_42688_set_wake_on_motion_threshold(uint8_t axis,uint8_t threshold)
{
  uint8_t bank = 4;
  icm_42688_write_reg(ICM42688_REG_BANK_SEL, &bank, 1);
  uint8_t womValue = threshold;
  if(axis == X_AXIS) {
    icm_42688_write_reg(ICM42688_ACCEL_WOM_X_THR, &womValue, 1);
  } else if(axis == Y_AXIS) {
    icm_42688_write_reg(ICM42688_ACCEL_WOM_Y_THR, &womValue, 1);
  } else if(axis == Z_AXIS) {
    icm_42688_write_reg(ICM42688_ACCEL_WOM_Z_THR, &womValue, 1);
  } else if(axis == ALL) {
    icm_42688_write_reg(ICM42688_ACCEL_WOM_X_THR, &womValue, 1);
    icm_42688_write_reg(ICM42688_ACCEL_WOM_Y_THR, &womValue, 1);
    icm_42688_write_reg(ICM42688_ACCEL_WOM_Z_THR, &womValue, 1);
  }
  delay(1);
  bank = 0;
  icm_42688_write_reg(ICM42688_REG_BANK_SEL, &bank, 1);
}
void icm_42688_set_wake_on_motion_interrupt(uint8_t axis)
{
  uint8_t bank = 0;
  icm_42688_write_reg(ICM42688_REG_BANK_SEL, &bank, 1);
  if(_INTPin == 1){
    icm_42688_write_reg(ICM42688_INT_SOURCE1, &axis, 1);
  } else {
    icm_42688_write_reg(ICM42688_INT_SOURCE4, &axis, 1);
  }
  delay(50);
  SMDConfig.SMDMode = 1;
  SMDConfig.WOMMode = 1;
  SMDConfig.WOMIntMode = 0;
  icm_42688_write_reg(ICM42688_SMD_CONFIG, &SMDConfig, 1);
}
void icm_42688_enable_smd_interrupt(uint8_t mode)
{
  uint8_t bank = 0;
  icm_42688_write_reg(ICM42688_REG_BANK_SEL, &bank, 1);
  uint8_t INT = 1<<3;
  if(mode != 0) {
    if(_INTPin == 1){
      icm_42688_write_reg(ICM42688_INT_SOURCE1, &INT, 1);
    } else {
      icm_42688_write_reg(ICM42688_INT_SOURCE4, &INT, 1);
    }
  }
  delay(50);
  SMDConfig.SMDMode = mode;
  SMDConfig.WOMMode = 1;
  SMDConfig.WOMIntMode = 0;
  icm_42688_write_reg(ICM42688_SMD_CONFIG, &SMDConfig, 1);
}

uint8_t icm_42688_read_interrupt_status(uint8_t reg)
{
  uint8_t bank = 0;
  uint8_t status = 0;
  icm_42688_write_reg(ICM42688_REG_BANK_SEL, &bank, 1);
  icm_42688_read_reg(reg, &status, 1);
  return status;
}

bool icm_42688_set_odr_fsr(uint8_t who,uint8_t ODR,uint8_t FSR)
{
  bool ret = true;
  uint8_t bank = 0;
  icm_42688_write_reg(ICM42688_REG_BANK_SEL, &bank, 1);
  if(who == GYRO){
    if(ODR > ODR_12_5KHZ || FSR > FSR_7){
      ret = false;
    }else{
      gyroConfig0.gyroODR = ODR;
      gyroConfig0.gyroFsSel = FSR;
      icm_42688_write_reg(ICM42688_GYRO_CONFIG0, &gyroConfig0, 1);
      switch(FSR){
        case FSR_0:
          _gyroRange = 4000/65535.0;
          break;
        case FSR_1:
          _gyroRange = 2000/65535.0;
          break;
        case FSR_2:
          _gyroRange = 1000/65535.0;
          break;
        case FSR_3:
          _gyroRange = 500/65535.0;
          break;
        case FSR_4:
          _gyroRange = 250/65535.0;
          break;
        case FSR_5:
          _gyroRange = 125/65535.0;
          break;
        case FSR_6:
          _gyroRange = 62.5/65535.0;
          break;
        case FSR_7:
          _gyroRange = 31.25/65535.0;
          break;
      }
    }
  } else if(who == ACCEL){
    if(ODR > ODR_500HZ || FSR > FSR_3){
      ret = false;
    } else{
      accelConfig0.accelODR = ODR;
      accelConfig0.accelFsSel = FSR;
      icm_42688_write_reg(ICM42688_ACCEL_CONFIG0, &accelConfig0, 1);
      switch(FSR){
        case FSR_0:
          _accelRange = 0.488f;
          break;
        case FSR_1:
          _accelRange = 0.244f;
          break;
        case FSR_2:
          _accelRange = 0.122f;
          break;
        case FSR_3:
          _accelRange = 0.061f;
          break;
      }
    }
  } 
  return ret;
}

void icm_42688_set_fifo_data_mode()
{
  uint8_t bank = 0;
  icm_42688_write_reg(ICM42688_REG_BANK_SEL, &bank, 1);
  FIFOConfig1.FIFOHiresEn = 0;
  FIFOConfig1.FIFOAccelEn = 1;
  FIFOConfig1.FIFOGyroEn = 1;
  FIFOConfig1.FIFOTempEn = 1;
  FIFOConfig1.FIFOTmstFsyncEn = 0;
  icm_42688_write_reg(ICM42688_FIFO_CONFIG1, &FIFOConfig1, 1);
}

void icm_42688_start_fifo_mode()
{
  uint8_t bank = 0;
  FIFOMode = true;
  icm_42688_write_reg(ICM42688_REG_BANK_SEL, &bank, 1);
  icm_42688_set_fifo_data_mode();
  uint8_t start = 1<<6;
  icm_42688_write_reg(ICM42688_FIFO_CONFIG, &start, 1);
  icm_42688_get_fifo_data();
}
void icm_42688_get_fifo_data()
{
  uint8_t data[16];
  icm_42688_read_reg(ICM42688_FIFO_DATA, data, 16);
  _accelX = (uint16_t)data[1]<<8 | (uint16_t)data[2];
  //DBG("_accelX");DBG(_accelX);
  _accelY = (uint16_t)data[3]<<8 | (uint16_t)data[4];
  //DBG("_accelY");DBG(_accelY);
  _accelZ = (uint16_t)data[5]<<8 | (uint16_t)data[6];
  //DBG("_accelZ");DBG(_accelZ);
  _gyroX = (uint16_t)data[7]<<8 | (uint16_t)data[8];
  //DBG("_gyroX");DBG(_gyroX);
  _gyroY = (uint16_t)data[9]<<8 | (uint16_t)data[10];
  //DBG("_gyroY");DBG(_gyroY);
  _gyroZ = (uint16_t)data[11]<<8 | (uint16_t)data[12];
  //DBG("_gyroZ");DBG(_gyroZ);
  _temp = (uint8_t)data[13];
  //DBG("_temp");DBG(data[13]);
}
void icm_42688_sotp_fifo_mode()
{
  uint8_t bank = 0;
  icm_42688_write_reg(ICM42688_REG_BANK_SEL, &bank, 1);
  uint8_t start = 1<<7;
  icm_42688_write_reg(ICM42688_FIFO_CONFIG, &start, 1);
}

void icm_42688_set_int_mode(uint8_t INTPin, uint8_t INTmode,uint8_t INTPolarity, uint8_t INTDriveCircuit)
{
  uint8_t bank = 0;
  icm_42688_write_reg(ICM42688_REG_BANK_SEL, &bank, 1);
  if(INTPin == 1){
    _INTPin = 1;
    INTConfig.INT1Mode = INTmode;
    INTConfig.INT1DriveCirCuit = INTDriveCircuit;
    INTConfig.INT1Polarity = INTPolarity;
  } else if(INTPin == 2){
    _INTPin = 2;
    INTConfig.INT2Mode = INTmode;
    INTConfig.INT2DriveCirCuit = INTDriveCircuit;
    INTConfig.INT2Polarity = INTPolarity;
  }
  icm_42688_write_reg(ICM42688_INT_CONFIG, &INTConfig, 1);
}

void icm_42688_start_temp_measure()
{
  PWRMgmt0.tempDis = 0;
  uint8_t bank = 0;
  icm_42688_write_reg(ICM42688_REG_BANK_SEL, &bank, 1);
  icm_42688_write_reg(ICM42688_PWR_MGMT0, &PWRMgmt0, 1);
  delay(1);
}
void icm_42688_start_gyro_measure(uint8_t mode)
{
  PWRMgmt0.gyroMode = mode;
  uint8_t bank = 0;
  icm_42688_write_reg(ICM42688_REG_BANK_SEL, &bank, 1);
  icm_42688_write_reg(ICM42688_PWR_MGMT0, &PWRMgmt0, 1);
  delay(1);
}

void icm_42688_start_accel_measure(uint8_t mode)
{
  PWRMgmt0.accelMode = mode;
  uint8_t bank = 0;
  icm_42688_write_reg(ICM42688_REG_BANK_SEL, &bank, 1);
  icm_42688_write_reg(ICM42688_PWR_MGMT0, &PWRMgmt0, 1);
  delay(10);
}

void icm_42688_set_gyro_notch_filter_frequency(double freq, uint8_t axis)
{
  uint8_t bank = 1;
  icm_42688_write_reg(ICM42688_REG_BANK_SEL, &bank, 1);
  double fdesired = freq * 1000;
  double coswz = cos(2*3.14*fdesired/32);
  int16_t nfCoswz;
  uint8_t nfCoswzSel;
  if(abs(coswz)<=0.875){
    nfCoswz = round(coswz*256);
    nfCoswzSel = 0;
  } else {
    nfCoswzSel = 1;
    if(coswz> 0.875){
      nfCoswz = round(8*(1-coswz)*256);
    } else if(coswz < -0.875){
      nfCoswz = round(-8*(1+coswz)*256);
    }
  }
  if(axis == X_AXIS){
    gyroConfigStatic9.gyroNFCoswzSelX = nfCoswzSel;
    gyroConfigStatic9.gyroNFCoswzX8 = nfCoswz>>8;
    icm_42688_write_reg(ICM42688_GYRO_CONFIG_STATIC6, &nfCoswz, 1);
    icm_42688_write_reg(ICM42688_GYRO_CONFIG_STATIC9, &gyroConfigStatic9, 1);
  } else if(axis == Y_AXIS){
    gyroConfigStatic9.gyroNFCoswzSelY = nfCoswzSel;
    gyroConfigStatic9.gyroNFCoswzY8 = nfCoswz>>8;
    icm_42688_write_reg(ICM42688_GYRO_CONFIG_STATIC7, &nfCoswz, 1);
    icm_42688_write_reg(ICM42688_GYRO_CONFIG_STATIC9, &gyroConfigStatic9, 1);
  } else if(axis == Z_AXIS){
    gyroConfigStatic9.gyroNFCoswzSelZ = nfCoswzSel;
    gyroConfigStatic9.gyroNFCoswzZ8 = nfCoswz>>8;
    icm_42688_write_reg(ICM42688_GYRO_CONFIG_STATIC8, &nfCoswz, 1);
    icm_42688_write_reg(ICM42688_GYRO_CONFIG_STATIC9, &gyroConfigStatic9, 1);
  } else if(axis == ALL)
  {
    gyroConfigStatic9.gyroNFCoswzSelX = nfCoswzSel;
    gyroConfigStatic9.gyroNFCoswzX8 = nfCoswz>>8;
    gyroConfigStatic9.gyroNFCoswzSelY = nfCoswzSel;
    gyroConfigStatic9.gyroNFCoswzY8 = nfCoswz>>8;
    gyroConfigStatic9.gyroNFCoswzSelZ = nfCoswzSel;
    gyroConfigStatic9.gyroNFCoswzZ8 = nfCoswz>>8;
    icm_42688_write_reg(ICM42688_GYRO_CONFIG_STATIC6, &nfCoswz, 1);
    icm_42688_write_reg(ICM42688_GYRO_CONFIG_STATIC7, &nfCoswz, 1);
    icm_42688_write_reg(ICM42688_GYRO_CONFIG_STATIC8, &nfCoswz, 1);
    icm_42688_write_reg(ICM42688_GYRO_CONFIG_STATIC9, &gyroConfigStatic9, 1);
  }
  bank = 0;
  icm_42688_write_reg(ICM42688_REG_BANK_SEL, &bank, 1);
}

void icm_42688_set_gyro_notch_filter_bandwidth(uint8_t bw)
{
  uint8_t bank = 1;
  icm_42688_write_reg(ICM42688_REG_BANK_SEL, &bank, 1);
  uint8_t bandWidth = (bw<<4) | 0x01;
  icm_42688_write_reg(ICM42688_GYRO_CONFIG_STATIC10, &bandWidth, 1);
  bank = 0;
  icm_42688_write_reg(ICM42688_REG_BANK_SEL, &bank, 1);
}

void icm_42688_set_gyro_notch_filter(bool mode)
{
  if(mode){
    gyroConfigStatic2.gyroNFDis = 0;
  } else {
    gyroConfigStatic2.gyroNFDis = 1;
  }
  uint8_t bank = 1;
  icm_42688_write_reg(ICM42688_REG_BANK_SEL, &bank, 1);
  icm_42688_write_reg(ICM42688_GYRO_CONFIG_STATIC2, &gyroConfigStatic2, 1);
  bank = 0;
  icm_42688_write_reg(ICM42688_REG_BANK_SEL, &bank, 1);
}
void icm_42688_set_aaf_bandwidth(uint8_t who, uint8_t BWIndex)
{
  uint8_t bank = 0;
  uint16_t AAFDeltsqr = BWIndex*BWIndex;
  if(who == GYRO){
    bank = 1;
    icm_42688_write_reg(ICM42688_REG_BANK_SEL, &bank, 1);
    icm_42688_write_reg(ICM42688_GYRO_CONFIG_STATIC3, &BWIndex, 1);
    icm_42688_write_reg(ICM42688_GYRO_CONFIG_STATIC4, &AAFDeltsqr, 1);
    gyroConfigStatic5.gyroAAFDeltsqr = AAFDeltsqr>>8;
    if(BWIndex == 1){
      gyroConfigStatic5.gyroAAFBitshift = 15;
    } else if(BWIndex == 2){
      gyroConfigStatic5.gyroAAFBitshift = 13;
    } else if(BWIndex == 3){
      gyroConfigStatic5.gyroAAFBitshift = 12;
    } else if(BWIndex == 4){
      gyroConfigStatic5.gyroAAFBitshift = 11;
    } else if(BWIndex == 5||BWIndex == 6){
      gyroConfigStatic5.gyroAAFBitshift = 10;
    } else if(BWIndex > 6 && BWIndex < 10){
      gyroConfigStatic5.gyroAAFBitshift = 9;
    } else if(BWIndex > 9 && BWIndex < 14){
      gyroConfigStatic5.gyroAAFBitshift = 8;
    } else if(BWIndex > 13 && BWIndex < 19){
      gyroConfigStatic5.gyroAAFBitshift = 7;
    } else if(BWIndex > 18 && BWIndex < 27){
      gyroConfigStatic5.gyroAAFBitshift = 6;
    } else if(BWIndex > 26 && BWIndex < 37){
      gyroConfigStatic5.gyroAAFBitshift = 5;
    } else if(BWIndex > 36 && BWIndex < 53){
      gyroConfigStatic5.gyroAAFBitshift = 4;
    } else if(BWIndex > 53 && BWIndex <= 63){
      gyroConfigStatic5.gyroAAFBitshift = 3;
    }
    icm_42688_write_reg(ICM42688_GYRO_CONFIG_STATIC5, &gyroConfigStatic5, 1);
    bank = 0;
    icm_42688_write_reg(ICM42688_REG_BANK_SEL, &bank, 1);
  } else if(who == ACCEL){
    bank = 2;
    icm_42688_write_reg(ICM42688_REG_BANK_SEL, &bank, 1);
    accelConfigStatic2.accelAAFDelt = BWIndex;
    icm_42688_write_reg(ICM42688_ACCEL_CONFIG_STATIC2, &accelConfigStatic2, 1);
    icm_42688_write_reg(ICM42688_ACCEL_CONFIG_STATIC3, &AAFDeltsqr, 1);
    accelConfigStatic4.accelAAFDeltsqr = AAFDeltsqr>>8;
    if(BWIndex == 1){
      accelConfigStatic4.accelAAFBitshift = 15;
    } else if(BWIndex == 2){
      accelConfigStatic4.accelAAFBitshift = 13;
    } else if(BWIndex == 3){
      accelConfigStatic4.accelAAFBitshift = 12;
    } else if(BWIndex == 4){
      accelConfigStatic4.accelAAFBitshift = 11;
    } else if(BWIndex == 5||BWIndex == 6){
      accelConfigStatic4.accelAAFBitshift = 10;
    } else if(BWIndex > 6 && BWIndex < 10){
      accelConfigStatic4.accelAAFBitshift = 9;
    } else if(BWIndex > 9 && BWIndex < 14){
      accelConfigStatic4.accelAAFBitshift = 8;
    } else if(BWIndex > 13 && BWIndex < 19){
      accelConfigStatic4.accelAAFBitshift = 7;
    } else if(BWIndex > 18 && BWIndex < 27){
      accelConfigStatic4.accelAAFBitshift = 6;
    } else if(BWIndex > 26 && BWIndex < 37){
      accelConfigStatic4.accelAAFBitshift = 5;
    } else if(BWIndex > 36 && BWIndex < 53){
      accelConfigStatic4.accelAAFBitshift = 4;
    } else if(BWIndex > 53 && BWIndex <= 63){
      accelConfigStatic4.accelAAFBitshift = 3;
    }
    icm_42688_write_reg(ICM42688_ACCEL_CONFIG_STATIC4, &accelConfigStatic4, 1);

    bank = 0;
    icm_42688_write_reg(ICM42688_REG_BANK_SEL, &bank, 1);
  } else if(who == ALL){
    bank = 1;
    icm_42688_write_reg(ICM42688_REG_BANK_SEL, &bank, 1);
    icm_42688_write_reg(ICM42688_GYRO_CONFIG_STATIC3, &BWIndex, 1);
    icm_42688_write_reg(ICM42688_GYRO_CONFIG_STATIC4, &AAFDeltsqr, 1);
    gyroConfigStatic5.gyroAAFDeltsqr = AAFDeltsqr>>8;
    if(BWIndex == 1){
      gyroConfigStatic5.gyroAAFBitshift = 15;
    } else if(BWIndex == 2){
      gyroConfigStatic5.gyroAAFBitshift = 13;
    } else if(BWIndex == 3){
      gyroConfigStatic5.gyroAAFBitshift = 12;
    } else if(BWIndex == 4){
      gyroConfigStatic5.gyroAAFBitshift = 11;
    } else if(BWIndex == 5||BWIndex == 6){
      gyroConfigStatic5.gyroAAFBitshift = 10;
    } else if(BWIndex > 6 && BWIndex < 10){
      gyroConfigStatic5.gyroAAFBitshift = 9;
    } else if(BWIndex > 9 && BWIndex < 14){
      gyroConfigStatic5.gyroAAFBitshift = 8;
    } else if(BWIndex > 13 && BWIndex < 19){
      gyroConfigStatic5.gyroAAFBitshift = 7;
    } else if(BWIndex > 18 && BWIndex < 27){
      gyroConfigStatic5.gyroAAFBitshift = 6;
    } else if(BWIndex > 26 && BWIndex < 37){
      gyroConfigStatic5.gyroAAFBitshift = 5;
    } else if(BWIndex > 36 && BWIndex < 53){
      gyroConfigStatic5.gyroAAFBitshift = 4;
    } else if(BWIndex > 53 && BWIndex <= 63){
      gyroConfigStatic5.gyroAAFBitshift = 3;
    }
    icm_42688_write_reg(ICM42688_GYRO_CONFIG_STATIC5, &gyroConfigStatic5, 1);
    bank = 2;
    icm_42688_write_reg(ICM42688_REG_BANK_SEL, &bank, 1);
    accelConfigStatic2.accelAAFDelt = BWIndex;
    icm_42688_write_reg(ICM42688_ACCEL_CONFIG_STATIC2, &accelConfigStatic2, 1);
    icm_42688_write_reg(ICM42688_ACCEL_CONFIG_STATIC3, &AAFDeltsqr, 1);
    accelConfigStatic4.accelAAFDeltsqr = AAFDeltsqr>>8;
    if(BWIndex == 1){
      accelConfigStatic4.accelAAFBitshift = 15;
    } else if(BWIndex == 2){
      accelConfigStatic4.accelAAFBitshift = 13;
    } else if(BWIndex == 3){
      accelConfigStatic4.accelAAFBitshift = 12;
    } else if(BWIndex == 4){
      accelConfigStatic4.accelAAFBitshift = 11;
    } else if(BWIndex == 5||BWIndex == 6){
      accelConfigStatic4.accelAAFBitshift = 10;
    } else if(BWIndex > 6 && BWIndex < 10){
      accelConfigStatic4.accelAAFBitshift = 9;
    } else if(BWIndex > 9 && BWIndex < 14){
      accelConfigStatic4.accelAAFBitshift = 8;
    } else if(BWIndex > 13 && BWIndex < 19){
      accelConfigStatic4.accelAAFBitshift = 7;
    } else if(BWIndex > 18 && BWIndex < 27){
      accelConfigStatic4.accelAAFBitshift = 6;
    } else if(BWIndex > 26 && BWIndex < 37){
      accelConfigStatic4.accelAAFBitshift = 5;
    } else if(BWIndex > 36 && BWIndex < 53){
      accelConfigStatic4.accelAAFBitshift = 4;
    } else if(BWIndex > 53 && BWIndex <= 63){
      accelConfigStatic4.accelAAFBitshift = 3;
    }
    icm_42688_write_reg(ICM42688_ACCEL_CONFIG_STATIC4, &accelConfigStatic4, 1);
    bank = 0;
    icm_42688_write_reg(ICM42688_REG_BANK_SEL, &bank, 1);
  }
}
void icm_42688_set_aaf(uint8_t who, bool mode)
{
  uint8_t bank = 0;
  if(who == GYRO){
    if(mode){
      gyroConfigStatic2.gyroAAFDis = 0;
    } else {
      gyroConfigStatic2.gyroAAFDis = 1;
    }
    bank = 1;
    icm_42688_write_reg(ICM42688_REG_BANK_SEL, &bank, 1);
    icm_42688_write_reg(ICM42688_GYRO_CONFIG_STATIC2, &gyroConfigStatic2, 1);
  }else if(who == ACCEL){
    if(mode){
      accelConfigStatic2.accelAAFDis = 0;
    } else {
      accelConfigStatic2.accelAAFDis = 1;
    }
    bank = 2;
    icm_42688_write_reg(ICM42688_REG_BANK_SEL, &bank, 1);
    icm_42688_write_reg(ICM42688_ACCEL_CONFIG_STATIC2, &accelConfigStatic2, 1);
  } else if(who == ALL){
    if(mode){
      gyroConfigStatic2.gyroAAFDis = 0;
      accelConfigStatic2.accelAAFDis = 0;
    } else {
      gyroConfigStatic2.gyroAAFDis = 1;
      accelConfigStatic2.accelAAFDis = 1;
    }
    bank = 1;
    icm_42688_write_reg(ICM42688_REG_BANK_SEL, &bank, 1);
    icm_42688_write_reg(ICM42688_GYRO_CONFIG_STATIC2, &gyroConfigStatic2, 1);
    bank = 2;
    icm_42688_write_reg(ICM42688_REG_BANK_SEL, &bank, 1);
    icm_42688_write_reg(ICM42688_ACCEL_CONFIG_STATIC2, &accelConfigStatic2, 1);
  }
  bank = 0;
  icm_42688_write_reg(ICM42688_REG_BANK_SEL, &bank, 1);
}

bool icm_42688_set_ui_filter(uint8_t who, uint8_t filterOrder, uint8_t UIFilterIndex)
{
  bool ret = true;
  uint8_t bank = 0;
  icm_42688_write_reg(ICM42688_REG_BANK_SEL, &bank, 1);
  if(filterOrder > 3 || UIFilterIndex > 15){
    ret = false;
  } else{
    if(who == GYRO) {
      gyroConfig1.gyroUIFiltODR = filterOrder;
      icm_42688_write_reg(ICM42688_GYRO_CONFIG1, &gyroConfig1, 1);
      gyroAccelConfig0.gyroUIFiltBW = UIFilterIndex;
      icm_42688_write_reg(ICM42688_GYRO_ACCEL_CONFIG0, &gyroAccelConfig0, 1);
    } else if(who == ACCEL){
      accelConfig1.accelUIFiltORD = filterOrder;
      icm_42688_write_reg(ICM42688_ACCEL_CONFIG1, &accelConfig1, 1);
      gyroAccelConfig0.accelUIFiltBW = UIFilterIndex;
      icm_42688_write_reg(ICM42688_GYRO_ACCEL_CONFIG0, &gyroAccelConfig0, 1);
    } else if(who == ALL){
      gyroConfig1.gyroUIFiltODR = filterOrder;
      icm_42688_write_reg(ICM42688_GYRO_CONFIG1, &gyroConfig1, 1);
      accelConfig1.accelUIFiltORD = filterOrder;
      icm_42688_write_reg(ICM42688_ACCEL_CONFIG1, &accelConfig1, 1);
      gyroAccelConfig0.gyroUIFiltBW = UIFilterIndex;
      gyroAccelConfig0.accelUIFiltBW = UIFilterIndex;
      icm_42688_write_reg(ICM42688_GYRO_ACCEL_CONFIG0, &gyroAccelConfig0, 1);
    }
  }
  return ret;
}

float icm_42688_get_range_accel()
{
  return _accelRange;
}
float icm_42688_get_range_gyro()
{
  return _gyroRange;
}

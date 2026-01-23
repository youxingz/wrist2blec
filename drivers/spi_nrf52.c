#include "icm_42688p.h"
#include "lis3mdl.h"

#define ENABLE_DEBUG

#include <zephyr/kernel.h>
#include <nrfx_gpiote.h>
#include <nrfx_spim.h>
#include <hal/nrf_gpio.h>

#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(spiNRF52, LOG_LEVEL_INF);


#define LIS3MDL_PIN_INT            NRF_GPIO_PIN_MAP(0, 4)
#define LIS3MDL_PIN_SPI_CS         NRF_GPIO_PIN_MAP(0, 20)
#define ICM42688P_PIN_INT          NRF_GPIO_PIN_MAP(0, 5)
#define ICM42688P_PIN_SPI_CS       NRF_GPIO_PIN_MAP(0, 17)
#define PIN_SPI_CLK      NRF_GPIO_PIN_MAP(0, 15)
#define PIN_SPI_MOSI     NRF_GPIO_PIN_MAP(0, 11)
#define PIN_SPI_MISO     NRF_GPIO_PIN_MAP(1, 9)

static nrfx_spim_t spim = NRFX_SPIM_INSTANCE(0);

static inline void lis3mdl_cs_select()
{
  nrf_gpio_pin_clear(LIS3MDL_PIN_SPI_CS);
}

static inline void lis3mdl_cs_unselect()
{
  nrf_gpio_pin_set(LIS3MDL_PIN_SPI_CS);
}

static inline void icm42688p_cs_select()
{
  nrf_gpio_pin_clear(ICM42688P_PIN_SPI_CS);
}

static inline void icm42688p_cs_unselect()
{
  nrf_gpio_pin_set(ICM42688P_PIN_SPI_CS);
}

int icm_42688_spi_init()
{
  nrfx_spim_config_t spim_config = NRFX_SPIM_DEFAULT_CONFIG(
    PIN_SPI_CLK,
    PIN_SPI_MOSI,
    PIN_SPI_MISO,
    ICM42688P_PIN_SPI_CS // PIN_SPI_CS
  ); // NRF_SPIM_PIN_NOT_CONNECTED
  spim_config.frequency = NRFX_MHZ_TO_HZ(8); // NRF_SPIM_FREQ_32M;

  nrfx_err_t err = nrfx_spim_init(&spim, &spim_config, NULL, NULL);
  if (err == NRFX_ERROR_INVALID_STATE)
  {
    LOG_WRN("Reinit again.");
    nrfx_spim_uninit(&spim);
    err = nrfx_spim_init(&spim, &spim_config, NULL, NULL);
  }
  
  if (err != NRFX_SUCCESS) {
    LOG_ERR("nrfx_spim_init() failed: 0x%08x", err);
  }

  nrf_gpio_cfg(
    ICM42688P_PIN_SPI_CS,
    NRF_GPIO_PIN_DIR_OUTPUT,
    NRF_GPIO_PIN_INPUT_DISCONNECT,
    NRF_GPIO_PIN_NOPULL,
    NRF_GPIO_PIN_H0H1,
    NRF_GPIO_PIN_NOSENSE);

  icm42688p_cs_unselect();

  return 0;
}
/**
 * @fn writeReg
 * @brief Write register function, design it as a virtual function, implemented by a derived class.
 * @param reg  Register address 8bits
 * @param pBuf Storage and buffer for data to be written
 * @param size Length of data to be written
 */
void icm_42688_write_reg(uint8_t reg, uint8_t* pBuf, uint8_t size)
{
  uint8_t tx_buf[size + 1];
  int index = 0;
  tx_buf[index++] = reg;
  for (int i = 0; i < size; i++) {
    tx_buf[index++] = pBuf[i];
  }
  nrfx_spim_xfer_desc_t xfer_desc = NRFX_SPIM_XFER_TX(tx_buf, index);
  icm42688p_cs_select();
  nrfx_err_t err = nrfx_spim_xfer(&spim, &xfer_desc, 0);
  icm42688p_cs_unselect();
  if (err != NRFX_SUCCESS) {
    // print error.
#ifdef ENABLE_DEBUG
    printk("nRF: SPI TXRX Error: %d\n", err);
#endif
  }
}

/**
 * @fn readReg
 * @brief Read register function, design it as a virtual function, implemented by a derived class.
 * @param reg  Register address 8bits
 * @param pBuf Read data storage and buffer
 * @param size Read data length
 * @return return the read length, returning 0 means length reading failed
 */
uint8_t icm_42688_read_reg(uint8_t reg, uint8_t* pBuf, uint8_t size)
{
  uint8_t tx_buf[size + 1], rx_buf[size + 1];
  int index = 0;
  tx_buf[index++] = reg | 0x80;
  for (int i = 0; i < size; i++) {
    tx_buf[index++] = pBuf[i];
  }
  nrfx_spim_xfer_desc_t xfer_desc = NRFX_SPIM_XFER_TRX(tx_buf, index, rx_buf, index);
  icm42688p_cs_select();
  nrfx_err_t err = nrfx_spim_xfer(&spim, &xfer_desc, 0);
  icm42688p_cs_unselect();
  if (err != NRFX_SUCCESS) {
    // print error.
#ifdef ENABLE_DEBUG
    printk("nRF: SPI TXRX Error: %d\n", err);
#endif
  }
  // copy result
  for (int i = 0; i < size; i++) {
    pBuf[i] = rx_buf[i + 1];
  }
  return size;
}


// LIS3MDL


lis3dml_err_t lis3dml_dev_init()
{
  nrf_gpio_cfg(
    LIS3MDL_PIN_SPI_CS,
    NRF_GPIO_PIN_DIR_OUTPUT,
    NRF_GPIO_PIN_INPUT_DISCONNECT,
    NRF_GPIO_PIN_NOPULL,
    NRF_GPIO_PIN_H0H1,
    NRF_GPIO_PIN_NOSENSE);

  nrf_gpio_cfg_input(LIS3MDL_PIN_INT, NRF_GPIO_PIN_NOPULL);
  lis3mdl_cs_unselect();
  return LIS3DML_SUCCESS;
}

lis3dml_err_t lis3dml_dev_uninit()
{
  return LIS3DML_SUCCESS;
}

lis3dml_err_t lis3dml_write_registers(uint8_t reg, uint8_t * pBuf, uint8_t size)
{
  uint8_t code = 0x00;
  // RW bit
  code |= 0 << 7;
  // MS bit
  code |= ((size == 1) ? 0 : 1) << 6;
  // Address
  code |= reg;

  uint8_t tx_buf[size + 1];
  int index = 0;
  tx_buf[index++] = code;
  for (int i = 0; i < size; i++) {
    tx_buf[index++] = pBuf[i];
  }
  nrfx_spim_xfer_desc_t xfer_desc = NRFX_SPIM_XFER_TX(tx_buf, index);
  lis3mdl_cs_select();
  nrfx_err_t err = nrfx_spim_xfer(&spim, &xfer_desc, 0);
  lis3mdl_cs_unselect();
  if (err != NRFX_SUCCESS) {
    // print error.
#ifdef ENABLE_DEBUG
    printk("nRF: SPI TXRX Error: %d\n", err);
#endif
  }
  return LIS3DML_SUCCESS;
}

lis3dml_err_t lis3dml_read_registers(uint8_t reg, uint8_t * pBuf, uint8_t size)
{
  uint8_t code = 0x00;
  // RW bit
  code |= 1 << 7;
  // MS bit
  code |= ((size == 1) ? 0 : 1) << 6;
  // Address
  code |= reg;

  uint8_t tx_buf[size + 1], rx_buf[size + 1];
  int index = 0;
  tx_buf[index++] = code;
  for (int i = 0; i < size; i++) {
    tx_buf[index++] = pBuf[i];
  }
  nrfx_spim_xfer_desc_t xfer_desc = NRFX_SPIM_XFER_TRX(tx_buf, index, rx_buf, index);
  lis3mdl_cs_select();
  nrfx_err_t err = nrfx_spim_xfer(&spim, &xfer_desc, 0);
  lis3mdl_cs_unselect();
  if (err != NRFX_SUCCESS) {
    // print error.
#ifdef ENABLE_DEBUG
    printk("nRF: SPI TXRX Error: %d\n", err);
#endif
  }
  // copy result
  for (int i = 0; i < size; i++) {
    pBuf[i] = rx_buf[i + 1];
  }
  return LIS3DML_SUCCESS;
}

uint8_t lis3dml_read_register(uint8_t address)
{
  uint8_t buffer[1];
  lis3dml_err_t err = lis3dml_read_registers(address, buffer, 1);
  if (err) return 0x00;

  return buffer[0];
}

lis3dml_err_t lis3dml_write_register(uint8_t address, uint8_t data)
{
  uint8_t buffer[1];
  buffer[0] = data;
  return lis3dml_write_registers(address, buffer, 1);
}

bool lis3dml_data_ready()
{
  return nrf_gpio_pin_read(LIS3MDL_PIN_INT); // high: available
}

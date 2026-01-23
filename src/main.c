#include <zephyr/kernel.h>
#include <zephyr/drivers/gpio.h>
#include <nrfx_spim.h>
#include <nrfx_gpiote.h>

#include "inc/ble.h"


#define MOSI_PIN NRF_GPIO_PIN_MAP(0,17)
#define MISO_PIN NRF_GPIO_PIN_MAP(0,15)
#define SCK_PIN  NRF_GPIO_PIN_MAP(1,9)
#define CS_PIN   NRF_GPIO_PIN_MAP(0,11)
static uint8_t m_tx_buffer[20];
static uint8_t m_rx_buffer[20];
static nrfx_spim_t spim_inst = NRFX_SPIM_INSTANCE(NRF_SPIM_INST_GET(0));

static const struct gpio_dt_spec led = GPIO_DT_SPEC_GET(DT_ALIAS(led0), gpios);

int main(void)
{
  if (!device_is_ready(led.port)) {
      while (1) { }  // 即使 GPIO 设备不 ready，也让 hb 变化
  }

  int ret =gpio_pin_configure_dt(&led, GPIO_OUTPUT_INACTIVE);
  if (ret) {
      while (1) { }  // 即使配置失败，也让 hb 变化
  }
  int err = 0;

  nrfx_spim_config_t spim_config = NRFX_SPIM_DEFAULT_CONFIG(SCK_PIN, MOSI_PIN, MISO_PIN, CS_PIN);

  err = nrfx_spim_init(&spim_inst, &spim_config, NULL, NULL);

  err = ble_init();
  // if (err)
  // {
  //   // LOG_ERR("Bluetooth adv init failed (err %d)", err);
  //   // return -2;
  // }
  // // LOG_INF("Bluetooth adv init success.");

  m_tx_buffer[0] = 0x01;
  while (1) {
    (void)gpio_pin_toggle_dt(&led);
    k_msleep(200);


    m_tx_buffer[0]++;
    nrfx_spim_xfer_desc_t spim_xfer_desc = NRFX_SPIM_XFER_TRX(m_tx_buffer, sizeof(m_tx_buffer),
                                                              m_rx_buffer, sizeof(m_rx_buffer));

    err = nrfx_spim_xfer(&spim_inst, &spim_xfer_desc, 0);
  }

  return 0;
}

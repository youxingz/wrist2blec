#include "inc/pba.h"

#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(pba, LOG_LEVEL_INF);

#define PIN_OUTPUT(pin) \
  nrf_gpio_cfg( \
    pin, \
    NRF_GPIO_PIN_DIR_OUTPUT, \
    NRF_GPIO_PIN_INPUT_DISCONNECT, \
    NRF_GPIO_PIN_NOPULL, \
    NRF_GPIO_PIN_H0H1, \
    NRF_GPIO_PIN_NOSENSE);


static int on_pba_charging(){}
static int on_pba_charged(){}
static int on_pba_button_press(){}

int pba_init()
{
  PIN_OUTPUT(PIN_POWER_EN);
  k_msleep(800); // 长按 800ms 后开机
  pba_power_en(true);

  PIN_OUTPUT(PIN_LED_R);
  PIN_OUTPUT(PIN_LED_G);
  PIN_OUTPUT(PIN_LED_B);

  PIN_OUTPUT(PIN_MOTOR_B_EN);
  PIN_OUTPUT(PIN_MOTOR_F_EN);

  // 开机后绿灯
  // pba_led_green(true);

  nrf_gpio_cfg_input(PIN_CHARGE_INT, NRF_GPIO_PIN_NOPULL);
  nrf_gpio_cfg_input(PIN_CHARGE_DONE, NRF_GPIO_PIN_NOPULL);
  nrf_gpio_cfg_input(PIN_BUTTON, NRF_GPIO_PIN_NOPULL);
  return 0;
}

bool pba_power_en(bool on)
{
  if (on) {
    nrf_gpio_pin_set(PIN_POWER_EN);
  } else {
    nrf_gpio_pin_clear(PIN_POWER_EN);
  }
  return true;
}

bool pba_led_green(bool en)
{
  return false;
  if (en) {
    nrf_gpio_pin_clear(PIN_LED_G);
  } else {
    nrf_gpio_pin_set(PIN_LED_G);
  }
  return true;
}

bool pba_led_blue(bool on)
{
  if (on) {
    nrf_gpio_pin_clear(PIN_LED_B);
  } else {
    nrf_gpio_pin_set(PIN_LED_B);
  }
  return true;
}
bool pba_led_red(bool on)
{
  if (on) {
    nrf_gpio_pin_clear(PIN_LED_R);
  } else {
    nrf_gpio_pin_set(PIN_LED_R);
  }
  return true;
}

bool pba_motor_front_en(bool on)
{
  if (on) {
    nrf_gpio_pin_set(PIN_MOTOR_F_EN);
  } else {
    nrf_gpio_pin_clear(PIN_MOTOR_F_EN);
  }
  return true;
}
bool pba_motor_back_en(bool on)
{
  if (on) {
    nrf_gpio_pin_set(PIN_MOTOR_B_EN);
  } else {
    nrf_gpio_pin_clear(PIN_MOTOR_B_EN);
  }
  return true;
}

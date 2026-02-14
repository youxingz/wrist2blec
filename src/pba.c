#include "inc/pba.h"

#include <zephyr/kernel.h>
#include <zephyr/sys/reboot.h>
#include <zephyr/irq.h>
#include <zephyr/logging/log.h>
#include <nrfx_gpiote.h>
LOG_MODULE_REGISTER(pba, LOG_LEVEL_INF);

#define PIN_OUTPUT(pin) \
  nrf_gpio_cfg( \
    pin, \
    NRF_GPIO_PIN_DIR_OUTPUT, \
    NRF_GPIO_PIN_INPUT_DISCONNECT, \
    NRF_GPIO_PIN_NOPULL, \
    NRF_GPIO_PIN_H0H1, \
    NRF_GPIO_PIN_NOSENSE);

static inline void pin_out_update(uint32_t pin, bool on) {
  if (on) {
    nrf_gpio_pin_clear(pin);
  } else {
    nrf_gpio_pin_set(pin);
  }
}

static int on_pba_charging(){}
static int on_pba_charged(){}
static int on_pba_button_press()
{
  // do nothing.
}
static int on_pba_button_long_press() // 长按关机
{
  event_before_shutdown();
  pba_power_en(false);
  sys_reboot(SYS_REBOOT_WARM);
  k_sleep(K_FOREVER);
}

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
  pba_led_green(false);
  pba_led_blue(false);
  pba_led_red(false);
  pba_motor_front_en(false);
  pba_motor_back_en(false);

  nrf_gpio_cfg_input(PIN_CHARGE_INT, NRF_GPIO_PIN_NOPULL);
  nrf_gpio_cfg_input(PIN_CHARGE_DONE, NRF_GPIO_PIN_NOPULL);
  nrf_gpio_cfg_input(PIN_BUTTON, NRF_GPIO_PIN_PULLUP);

  return event_after_startup();
  // return 0;
}

// 10ms 一次刷新
int pba_loop()
{
  static int64_t press_time_ms = -1;
  int64_t now = k_uptime_get();
  int level = nrf_gpio_pin_read(PIN_BUTTON); // 0=按下(接地), 1=松开(上拉)


  if (level == 1) { // 松开/平时状态
    if (press_time_ms >= 0) {
      int64_t duration_ms = now - press_time_ms;
      if (duration_ms >= 1000) {
        // long press.
        on_pba_button_long_press();
      } else {
        on_pba_button_press();
      }
    }
    press_time_ms = -1;
    return 0;
  }

  if (level == 0) { // pressed.
    if (press_time_ms == -1) {
      press_time_ms = now; // 刚按下
      return 0;
    } else {
      int64_t duration_ms = now - press_time_ms;
      if (duration_ms >= 1000) {
        // long press.
        on_pba_button_long_press();
        return 0;
      }
    }
  }

  return 0;
}

bool pba_power_en(bool on)
{
  if (on) {
    nrf_gpio_pin_set(PIN_POWER_EN);
  } else {
    nrf_gpio_pin_clear(PIN_POWER_EN);
    // 关闭所有 LED 灯
    pba_led_green(false);
    pba_led_blue(false);
    pba_led_red(false);
    // 关闭所有电机
    pba_motor_front_en(false);
    pba_motor_back_en(false);
  }
  return true;
}

bool pba_led_green(bool on)
{
  pin_out_update(PIN_LED_G, on);
  return true;
}

bool pba_led_blue(bool on)
{
  pin_out_update(PIN_LED_B, on);
  return true;
}
bool pba_led_red(bool on)
{
  pin_out_update(PIN_LED_R, on);
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

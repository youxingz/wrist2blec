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


static int on_pba_charging(){}
static int on_pba_charged(){}
static int on_pba_button_press()
{
  static int64_t press_time_ms = -1;
  int64_t now = k_uptime_get();
  int level = nrf_gpio_pin_read(PIN_BUTTON); // 0=按下(接地), 1=松开(上拉)

  if (level == 0) {
    press_time_ms = now;
    return 0;
  }

  if (press_time_ms >= 0) {
    int64_t duration_ms = now - press_time_ms;
    press_time_ms = -1;
    if (duration_ms >= 2000) {
      // LOG_INF("button long press: %lld ms", duration_ms);
      // 关蓝牙+关机
      event_before_shutdown();
      pba_power_en(false);
      // 此处系统应当挂掉，如果没有，则自动重启即可：
      sys_reboot(SYS_REBOOT_WARM);
    } else {
      // LOG_INF("button short press: %lld ms", duration_ms);
      // 短按逻辑暂不处理
    }
  }
  return 0;
}

static nrfx_gpiote_t gpiote_inst = NRFX_GPIOTE_INSTANCE(NRF_GPIOTE_INST_GET(0));

static void pba_button_isr(nrfx_gpiote_pin_t pin,
                           nrfx_gpiote_trigger_t trigger,
                           void *p_context)
{
  ARG_UNUSED(pin);
  ARG_UNUSED(trigger);
  ARG_UNUSED(p_context);
  on_pba_button_press();
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

  if (!nrfx_gpiote_init_check(&gpiote_inst)) {
    int err = nrfx_gpiote_init(&gpiote_inst, NRFX_GPIOTE_DEFAULT_CONFIG_IRQ_PRIORITY);
    if (err != 0 && err != -EALREADY) {
      LOG_ERR("gpiote init failed: %d", err);
      return -EIO;
    }
  }

  static const nrf_gpio_pin_pull_t button_pull = NRF_GPIO_PIN_PULLUP;
  static const nrfx_gpiote_trigger_config_t button_trigger = {
    .trigger = NRFX_GPIOTE_TRIGGER_TOGGLE,
    .p_in_channel = NULL,
  };
  static const nrfx_gpiote_handler_config_t button_handler = {
    .handler = pba_button_isr,
    .p_context = NULL,
  };
  static const nrfx_gpiote_input_pin_config_t button_cfg = {
    .p_pull_config = &button_pull,
    .p_trigger_config = &button_trigger,
    .p_handler_config = &button_handler,
  };

  int err = nrfx_gpiote_input_configure(&gpiote_inst, PIN_BUTTON, &button_cfg);
  if (err != 0) {
    LOG_ERR("button init failed: %d", err);
    return -EIO;
  }

  nrfx_gpiote_trigger_enable(&gpiote_inst, PIN_BUTTON, true);

  return event_after_startup();
  // return 0;
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

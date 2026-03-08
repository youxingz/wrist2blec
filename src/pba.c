#include "inc/pba.h"

#include <zephyr/kernel.h>
#include <zephyr/sys/reboot.h>
#include <zephyr/irq.h>
#include <zephyr/logging/log.h>
#include <nrfx_gpiote.h>
LOG_MODULE_REGISTER(pba, LOG_LEVEL_INF);


static led_status_t led_current_status = LED_POWER_OFF;
static bool led_status_ended = true;
void pba_led_status_update(led_status_t new_status) {
  // 根据优先级进行排序，如果有高优先级的在进行，则低优先级无法打断
  if (new_status > led_current_status) { // && !led_status_ended
    led_current_status = new_status;
    led_status_ended = true; // 强制结束当前状态
  } else {
    led_current_status = new_status; // 弱更新，等待当前状态结束后再更新显示
  }
}

static int64_t last_wtg_alert  = 0;
static int64_t last_wtg_stable = 0;
bool pba_trigger_wtg_alert()
{
  last_wtg_alert = k_uptime_get();
  return true;
}
bool pba_trigger_wtg_stable()
{
  last_wtg_stable = k_uptime_get();
  return true;
}

static void hanlde_wtg(int64_t now) {
  if (last_wtg_alert == 0) {
    last_wtg_alert = now;
  }
  if (last_wtg_stable == 0) {
    last_wtg_stable = now;
  }
  if (now - last_wtg_alert > 600000) { // 10min
    pba_power_off();
  }
  if (now - last_wtg_stable > 180000) { // 3min
    pba_power_off();
  }
}

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
  pba_led_status_update(LED_POWER_OFF);
  led_status_ended = true; // force update led status.
  k_msleep(2000); // 长按 2s 后关机
  pba_power_off();
}

bool pba_power_off()
{
  event_before_shutdown();
  pba_power_en(false);
  sys_reboot(SYS_REBOOT_WARM);
  k_sleep(K_FOREVER);
  return true;
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

  pba_led_status_update(LED_POWER_ON);

  return event_after_startup();
  // return 0;
}

static void handle_on_button(int64_t now) {
  static int64_t press_time_ms = -1;

  // Button:
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
}

static void handle_on_charger() {
  bool charging = nrf_gpio_pin_read(PIN_CHARGE_INT) == 0; // 0=充电中, 1=未充电
  bool charged = nrf_gpio_pin_read(PIN_CHARGE_DONE) == 0;
  if (charging) {
    on_pba_charging();
    return;
  }
  if (charged) {
    on_pba_charged();
    return;
  }
}

static void handle_on_pba_led_status_update(int64_t now) {
  static led_status_t real_status;
  static int64_t led_low_power_from = 0;
  static int64_t led_power_on_from = 0;
  static int64_t led_power_off_from = 0;

  if (led_status_ended) {
    return;
  }
  if (real_status != led_current_status) {
    real_status = led_current_status;
    led_status_ended = false;
  }
  switch(real_status) {
    case LED_LOW_POWER:
    {
      // 每 10 秒红灯闪烁一次(9+1)
      if (led_low_power_from == 0) {
        led_low_power_from = now;
        return;
      }
      if (now - led_low_power_from >= 9000) {
        pba_led_red(false);
        return;
      }
      if (now - led_low_power_from >= 10000) {
        pba_led_red(true);
        return;
      }
      if (now - led_low_power_from >= 11000) {
        pba_led_red(false);
        led_low_power_from = 0;
        led_status_ended = true;
        return;
      }
      break;
    }
    case LED_POWER_ON:
    {
      // 绿灯快闪2秒后进入常态呼吸 + 短振一下
      if (led_power_on_from == 0) {
        led_power_on_from = now;
        return;
      }
      if (now - led_power_on_from < 2000) {
        // 快闪
        if (((now - led_power_on_from) / 200) % 2 == 0) {
          pba_led_green(false);
        } else {
          pba_led_green(true);
        }
        // 短震
        if (now - led_power_on_from >= 1000) {
          pba_motor_front_en(true);
        } else {
          pba_motor_front_en(false);
        }
      } else {
        // 常亮+禁震
        pba_motor_front_en(false);
        pba_led_green(true);
        led_power_on_from = 0;
        led_status_ended = true;
      }
      break;
    }
    case LED_POWER_OFF:
    {
      // 红灯快闪2秒后熄灭+ 短振一下
      if (now - led_power_off_from < 2000) {
        // 快闪
        if (((now - led_power_off_from) / 200) % 2 == 0) {
          pba_led_red(false);
        } else {
          pba_led_red(true);
        }
        // 短震
        if (now - led_power_off_from >= 1000) {
          pba_motor_front_en(true);
        } else {
          pba_motor_front_en(false);
        }
      } else {
        // 常亮+禁震
        pba_motor_front_en(false);
        pba_led_red(false);
        pba_led_blue(false);
        pba_led_green(false);
        led_power_off_from = 0;
        led_status_ended = true;
      }
      break;
    }
    case LED_CHARGING:
    {
      // 红色常亮
      pba_led_red(true);
      led_status_ended = true;
      break;
    }
    case LED_CHARGED:
    {
      // 绿色常亮
      pba_led_green(true);
      led_status_ended = true;
      break;
    }
    case LED_BAD_GES:
    {
      // 红色闪烁
      if (((now) / 500) % 2 == 0) {
        pba_led_red(false);
      } else {
        pba_led_red(true);
      }
      led_status_ended = true;
      break;
    }
    case LED_OTA:
    {
      // 蓝色闪烁
      if (((now) / 500) % 2 == 0) {
        pba_led_blue(false);
      } else {
        pba_led_blue(true);
      }
      led_status_ended = true;
      break;
    }
    case LED_GOOD_GES:
    {
      // 绿色闪烁
      if (((now) / 500) % 2 == 0) {
        pba_led_green(false);
      } else {
        pba_led_green(true);
      }
      led_status_ended = true;
      break;
    }
  }
}

// 10ms 一次刷新
int pba_loop()
{
  int64_t now = k_uptime_get();
  // button:
  handle_on_button(now);

  // charger:
  handle_on_charger();

  // led:
  handle_on_pba_led_status_update(now);

  // wtg:
  hanlde_wtg(now);

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

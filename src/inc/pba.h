#pragma once

#include <hal/nrf_gpio.h>

#define PIN_LED_R NRF_GPIO_PIN_MAP(0, 0)
#define PIN_LED_G NRF_GPIO_PIN_MAP(0, 1)
#define PIN_LED_B NRF_GPIO_PIN_MAP(0, 10)

#define PIN_POWER_EN  NRF_GPIO_PIN_MAP(0, 30)
#define PIN_BUTTON    NRF_GPIO_PIN_MAP(0, 29)

#define PIN_MOTOR_F_EN  NRF_GPIO_PIN_MAP(0, 28)
#define PIN_MOTOR_B_EN  NRF_GPIO_PIN_MAP(0, 2)

#define PIN_CHARGE_INT  NRF_GPIO_PIN_MAP(0, 3)
#define PIN_CHARGE_DONE NRF_GPIO_PIN_MAP(0, 9)


typedef enum {
  LED_OTA = 0,
  LED_RESET_PR,
  LED_LOW_POWER,
  LED_POWER_ON,
  LED_POWER_OFF,
  LED_BAD_GES,
  LED_GOOD_GES,
  LED_CHARGING,
  LED_CHARGED,
} led_status_t;
void pba_led_status_update(led_status_t new_status);

int pba_init();
int pba_loop();
bool pba_power_en(bool on);
bool pba_power_off();

bool pba_led_blue(bool on);
bool pba_led_green(bool on);
bool pba_led_red(bool on);

bool pba_motor_en(bool on, bool double_motor_mode);
bool pba_motor_front_en(bool on);
bool pba_motor_back_en(bool on);

bool pba_trigger_wtg_alert();
bool pba_trigger_wtg_stable();
bool pba_trigger_wtg_0p8_motor_stable();

int event_after_startup();
int event_before_shutdown();

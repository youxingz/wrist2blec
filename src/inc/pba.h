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


int pba_init();
bool pba_power_en(bool on);

bool pba_led_blue(bool on);
bool pba_led_green(bool on);
bool pba_led_red(bool on);

bool pba_motor_front_en(bool on);
bool pba_motor_back_en(bool on);

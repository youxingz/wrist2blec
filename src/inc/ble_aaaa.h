#ifndef BLE_AAAA_H__
#define BLE_AAAA_H__

#include <stdint.h>

int ble_aaaa_init();

int ble_aaff_notify_commit(uint8_t * data, uint8_t len);

int ble_aaef_notify_commit(uint8_t * data, uint8_t len);

#endif

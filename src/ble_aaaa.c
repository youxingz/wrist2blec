#ifndef ENABLE_FACA_SERVICE__
#define ENABLE_FACA_SERVICE__

#include "inc/ble_aaaa_priv.h"

#include <zephyr/bluetooth/bluetooth.h>
#include <zephyr/bluetooth/hci.h>
#include <zephyr/bluetooth/conn.h>
#include <zephyr/bluetooth/uuid.h>
#include <zephyr/bluetooth/gatt.h>

#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(aaaa, LOG_LEVEL_INF);


/*
 **************************************************
 *                                                *
 *  Bluetooth LE.                                 *
 *                                                *
 **************************************************
 */

// SERVICE: AAAA
struct bt_uuid_128 aaaa_service_uuid = BT_UUID_INIT_128(BT_UUID_128_ENCODE(0x0000AAAA, 0x0000, 0x1000, 0x8000, 0x00805f9b34fb));

// EMG/ECG/EOG/EEG Characteristics
//// AAF0: READ STATE
struct bt_uuid_128 aaf0_char_uuid = BT_UUID_INIT_128(BT_UUID_128_ENCODE(0x0000AAF0, 0x0000, 0x1000, 0x8000, 0x00805f9b34fb));
//// AAFF: STREAM
struct bt_uuid_128 aaff_char_uuid = BT_UUID_INIT_128(BT_UUID_128_ENCODE(0x0000AAFF, 0x0000, 0x1000, 0x8000, 0x00805f9b34fb));

// IMU/MAG Characteristics
//// AAE0: READ STATE
struct bt_uuid_128 aae0_char_uuid = BT_UUID_INIT_128(BT_UUID_128_ENCODE(0x0000AAE0, 0x0000, 0x1000, 0x8000, 0x00805f9b34fb));
//// AAEF: STREAM
struct bt_uuid_128 aaef_char_uuid = BT_UUID_INIT_128(BT_UUID_128_ENCODE(0x0000AAEF, 0x0000, 0x1000, 0x8000, 0x00805f9b34fb));

// PPG Characteristics
//// AAD0: READ STATE
struct bt_uuid_128 aad0_char_uuid = BT_UUID_INIT_128(BT_UUID_128_ENCODE(0x0000AAD0, 0x0000, 0x1000, 0x8000, 0x00805f9b34fb));
//// AADF: STREAM
struct bt_uuid_128 aadf_char_uuid = BT_UUID_INIT_128(BT_UUID_128_ENCODE(0x0000AADF, 0x0000, 0x1000, 0x8000, 0x00805f9b34fb));


BT_GATT_SERVICE_DEFINE(aaaa_service,
  BT_GATT_PRIMARY_SERVICE(&aaaa_service_uuid),
  /** IMU/MAG **/
  // State.
  BT_GATT_CHARACTERISTIC(&aae0_char_uuid.uuid, BT_GATT_CHRC_WRITE | BT_GATT_CHRC_WRITE_WITHOUT_RESP | BT_GATT_CHRC_NOTIFY | BT_GATT_CHRC_READ,
            BT_GATT_PERM_READ | BT_GATT_PERM_WRITE, ble_on_aae0_read_request, ble_on_aae0_write_request, NULL),
  BT_GATT_CCC(ble_ccc_cfg_changed_aae0, BT_GATT_PERM_READ | BT_GATT_PERM_WRITE),
  // Stream.
  BT_GATT_CHARACTERISTIC(&aaef_char_uuid.uuid, BT_GATT_CHRC_NOTIFY | BT_GATT_CHRC_READ,
            BT_GATT_PERM_READ | BT_GATT_PERM_WRITE, NULL, NULL, NULL),
  BT_GATT_CCC(ble_ccc_cfg_changed_aaef, BT_GATT_PERM_READ | BT_GATT_PERM_WRITE),
);


/*
 **************************************************
 *                                                *
 *  Worker Loop                                 *
 *                                                *
 **************************************************
 */


// Looper

#define LOOPER_INTERVAL         1     // 1ms

static void looper_work_handler(struct k_work *work);

static K_WORK_DELAYABLE_DEFINE(looper_work, looper_work_handler);

static void looper_work_handler(struct k_work *work)
{
  ble_aaef_loop();
	k_work_reschedule(k_work_delayable_from_work(work), K_MSEC(LOOPER_INTERVAL));
}

int ble_aaaa_init()
{
  int err = 0;
  // err = k_work_schedule(&looper_work, K_NO_WAIT);
  err = k_work_schedule(&looper_work, K_MSEC(200)); // 延迟 200ms 启动，确保尽可能在最后时刻
  return 0;
}

#endif

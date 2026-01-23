#include "inc/ble.h"

#include <zephyr/sys/util.h>
#include <zephyr/settings/settings.h>
#include <zephyr/bluetooth/bluetooth.h>
#include <zephyr/bluetooth/gatt.h>

#include "inc/ble_conn.h"
#include "inc/ble_aaaa.h"

void mtu_updated(struct bt_conn *conn, uint16_t tx, uint16_t rx)
{
	printk("Updated MTU: TX: %d RX: %d bytes\n", tx, rx);
}
static struct bt_gatt_cb gatt_callbacks = {.att_mtu_updated = mtu_updated};

int ble_init()
{
  int err = 0;
  
  // Bluetooth
  err = bt_enable(NULL);
  if (err)
  {
    // LOG_ERR("Bluetooth init failed (err %d)", err);
    // return -1;
  }
  bt_gatt_cb_register(&gatt_callbacks);
  // // // LOG_INF("Bluetooth init success.");
  // // if (IS_ENABLED(CONFIG_BT_SETTINGS))
  // // {
  // //   settings_load();
  // //   // LOG_INF("Settings init success.");
  // // }

  err = ble_aaaa_init();
  if (err) { return err; }
  
  err = ble_adv_init();
  if (err) { return err; }
  
  err = ble_adv_start();
  if (err) { return err; }

  return 0;
}

int ble_uninit()
{
  return 0;
}

#ifndef BLE_AAAA_AAE_IMU_C__
#define BLE_AAAA_AAE_IMU_C__

#include "inc/ble_aaaa_priv.h"
#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(AAF, LOG_LEVEL_INF);

#define STATE_CODE_LEN 20
#define STREAM_BUFFER_LEN 512

typedef struct {
  uint8_t state_codes[STATE_CODE_LEN];
  uint8_t stream_buffer[STREAM_BUFFER_LEN];
  volatile bool stream_ready;
  // Notifications:
  volatile bool aaf0_notify_enabled;
  volatile bool aaff_notify_enabled;
} aaf_t;

static volatile aaf_t state_aaf;

void ble_ccc_cfg_changed_aaf0(const struct bt_gatt_attr *attr, uint16_t value)
{
	ARG_UNUSED(attr);
	state_aaf.aaf0_notify_enabled = (value == BT_GATT_CCC_NOTIFY);
	LOG_INF("Notification[AAF0] %s", state_aaf.aaf0_notify_enabled ? "enabled" : "disabled");
}

void ble_ccc_cfg_changed_aaff(const struct bt_gatt_attr *attr, uint16_t value)
{
	ARG_UNUSED(attr);
	state_aaf.aaff_notify_enabled = (value == BT_GATT_CCC_NOTIFY);
	LOG_INF("Notification[AAFF] %s", state_aaf.aaff_notify_enabled ? "enabled" : "disabled");
}

ssize_t ble_on_aaf0_read_request(struct bt_conn *conn, const struct bt_gatt_attr *attr, void *buf, uint16_t len, uint16_t offset)
{
  // const char *value = (const char *)state_aaf.state_codes;
  // return bt_gatt_attr_read(conn, attr, buf, len, offset, value, STATE_CODE_LEN);

  // TEST CODE (Read from SPI):


  const char *value = (const char *)state_aaf.state_codes;
  return bt_gatt_attr_read(conn, attr, buf, len, offset, value, STATE_CODE_LEN);
}

ssize_t ble_on_aaf0_write_request(struct bt_conn *conn, const struct bt_gatt_attr *attr, const void *buf, uint16_t len, uint16_t offset, uint8_t flags)
{

  LOG_INF("payload len: %d", len);
  if (len == 1) { // debug mode
    int err = 0; // xx_verify();
    LOG_INF("faca: debug verify: %02x", err);
    return 0;
  }
  /**
   * 
   * Request Payload Parser & Resp.
   * 
  */
  uint8_t * data = (uint8_t *)buf;
  uint16_t request_id = (uint16_t)(data[0] << 8) | (data[1] & 0xFF);
  uint8_t method = data[2] & 0xFF;

  return len;
}

int ble_aaff_notify_commit(uint8_t * data, uint8_t len)
{
  return 0;
}

int ble_aaff_loop()
{
  // if ready
  if (!state_aaf.aaff_notify_enabled) {
    return -1;
  }
  if (!state_aaf.stream_ready) {
    return -2;
  }
  int err = bt_gatt_notify(NULL, &aaaa_service.attrs[EEG_AAFF_INDEX], (uint8_t*)&(state_aaf.stream_buffer), (STREAM_BUFFER_LEN + 1));
  if (err) {
    return err;
  }
  return 0;
}

#endif

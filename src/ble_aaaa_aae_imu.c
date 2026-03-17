#ifndef BLE_AAAA_AAE_IMU_C__
#define BLE_AAAA_AAE_IMU_C__

#include "inc/ble_aaaa_priv.h"
#include "inc/persistence.h"
#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(AAE, LOG_LEVEL_INF);

#define STATE_CODE_LEN 20
#define STREAM_BUFFER_LEN 38

typedef struct {
  uint8_t state_codes[STATE_CODE_LEN];
  uint8_t stream_buffer[STREAM_BUFFER_LEN];
  volatile bool stream_ready;
  // Notifications:
  volatile bool aae0_notify_enabled;
  volatile bool aaef_notify_enabled;
} aae_t;

static volatile aae_t state_aae;

void ble_ccc_cfg_changed_aae0(const struct bt_gatt_attr *attr, uint16_t value)
{
	ARG_UNUSED(attr);
	state_aae.aae0_notify_enabled = (value == BT_GATT_CCC_NOTIFY);
	LOG_INF("Notification[aae0] %s", state_aae.aae0_notify_enabled ? "enabled" : "disabled");
}

void ble_ccc_cfg_changed_aaef(const struct bt_gatt_attr *attr, uint16_t value)
{
	ARG_UNUSED(attr);
	state_aae.aaef_notify_enabled = (value == BT_GATT_CCC_NOTIFY);
	LOG_INF("Notification[aaeF] %s", state_aae.aaef_notify_enabled ? "enabled" : "disabled");
}

ssize_t ble_on_aae0_read_request(struct bt_conn *conn, const struct bt_gatt_attr *attr, void *buf, uint16_t len, uint16_t offset)
{
  float yaw = 0.0f;
  float pitch = 0.0f;
  float roll = 0.0f;
  // alg_posture_get_thresholds(&yaw, &pitch, &roll);
  int err = presistence_get(PERSISTENCE_KEY_YAW_THRESHOLD, (uint8_t *)&yaw);
  if (err < 0) {
    LOG_INF("persistence get yaw threshold failed (err %d)", err);
  }
  err = presistence_get(PERSISTENCE_KEY_PITCH_THRESHOLD, (uint8_t *)&pitch);
  if (err < 0) {
    LOG_INF("persistence get pitch threshold failed (err %d)", err);
  }
  err = presistence_get(PERSISTENCE_KEY_ROLL_THRESHOLD, (uint8_t *)&roll);
  if (err < 0) {
    LOG_INF("persistence get roll threshold failed (err %d)", err);
  }
  uint8_t value[3] = {
    (uint8_t)((int)yaw),
    (uint8_t)((int)pitch),
    (uint8_t)((int)roll),
  };
  return bt_gatt_attr_read(conn, attr, buf, len, offset, value, sizeof(value));
}

ssize_t ble_on_aae0_write_request(struct bt_conn *conn, const struct bt_gatt_attr *attr, const void *buf, uint16_t len, uint16_t offset, uint8_t flags)
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

  if (method == 0x02) {
    if (len < 5) {
      LOG_INF("cmd 0x02 payload too short, req: %u", request_id);
      return len;
    }

    presistence_key_t axis = (presistence_key_t)(data[3] & 0xFF);
    uint8_t threshold = data[4] & 0xFF;
    const char *axis_name = NULL;

    switch (axis) {
      case PERSISTENCE_KEY_YAW_THRESHOLD:
        axis_name = "yaw";
        break;
      case PERSISTENCE_KEY_PITCH_THRESHOLD:
        axis_name = "pitch";
        break;
      case PERSISTENCE_KEY_ROLL_THRESHOLD:
        axis_name = "roll";
        break;
      default:
        LOG_INF("cmd 0x02 invalid axis: 0x%02x", axis);
        return len;
    }

    int err = presistence_upsert(axis, threshold);
    if (err < 0) {
      LOG_INF("cmd 0x02 storage write failed (err %d)", err);
    } else {
      LOG_INF("cmd 0x02 saved %s threshold: %u", axis_name, threshold);
    }
  }

  if (method == 0x03) {
    // config.
    int index = 3;
    // uint8_t axis_x = data[index++] & 0xFF;       // x轴
    // uint8_t axis_y = data[index++] & 0xFF;       // y轴
    // uint8_t axis_z = data[index++] & 0xFF;       // z轴
    uint8_t motor_mode = data[index++] & 0xFF;     // 电机模式：单/双
    uint8_t hand_mode  = data[index++] & 0xFF;     // 手方向
    uint8_t precision_mode = data[index++] & 0xFF; // 容忍度数
    uint8_t viber_mode = data[index++] & 0xFF;     // 震动模式
    presistence_upsert(PERSISTENCE_KEY_MOTOR_MODE, motor_mode);
    presistence_upsert(PERSISTENCE_KEY_HAND_MODE, hand_mode);
    presistence_upsert(PERSISTENCE_KEY_PRECISION_MODE, precision_mode);
    presistence_upsert(PERSISTENCE_KEY_VIBER_MODE, viber_mode);
  }
  return len;
}

int ble_aaef_notify_commit(uint8_t * data, uint8_t len)
{
  // notify
  if (len <= 0) return -1;
  if (!data) return -1;
  memcpy(state_aae.stream_buffer, data, len);
  state_aae.stream_ready = true;
  return 0;
}

int ble_aaef_init()
{
  return 0;
}

int ble_aaef_loop()
{
  // if device connected.
  if (!state_aae.aaef_notify_enabled) {
    return -1;
  }
  // if ready.
  if (!state_aae.stream_ready) {
    return -2;
  }
  state_aae.stream_ready = false;
  int err = bt_gatt_notify(NULL, &aaaa_service.attrs[IMU_AAEF_INDEX], (uint8_t*)&(state_aae.stream_buffer), (STREAM_BUFFER_LEN));
  if (err) {
    // try again once.
    err = bt_gatt_notify(NULL, &aaaa_service.attrs[IMU_AAEF_INDEX], (uint8_t*)&(state_aae.stream_buffer), (STREAM_BUFFER_LEN));
    return err;
  }
  return 0;
}

#endif

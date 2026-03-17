#include "inc/persistence.h"
#include "inc/storage.h"
#include "inc/alg_posture.h"
#include <zephyr/logging/log.h>
#include <errno.h>

LOG_MODULE_REGISTER(presistence, LOG_LEVEL_INF);

static int pre_cache[PERSISTENCE_KEY_LEN];

int presistence_init()
{
  // init config here.
  for (int i = 0; i < PERSISTENCE_KEY_LEN; i++) {
    uint8_t value;
    int err = storage_read(i, &value);
    if (err < 0) {
      // LOG_INF("persistence read key %d failed (err %d)", i, err);
      pre_cache[i] = err; // default value: err code.
    } else {
      pre_cache[i] = value;
      // LOG_INF("persistence read key %d: %u", i, value);
    }
  }
  return 0;
}

int presistence_upsert(presistence_key_t key, uint8_t value)
{
  pre_cache[key] = value;
  
  // axis:
  if (key >= PERSISTENCE_KEY_YAW_THRESHOLD && key <= PERSISTENCE_KEY_ROLL_THRESHOLD) {
    // update alg threshold immediately.
    int alg_err = alg_posture_update_threshold(key, (float)value);
    if (alg_err < 0) {
      LOG_INF("persistence update alg threshold failed (err %d)", alg_err);
    }
  }
  return storage_upsert(key, value);
}

int presistence_get(presistence_key_t key, uint8_t *value)
{
  int v = pre_cache[key]; // must be a uint8_t.
  if (v >= 0) {
    return v;
  }
  int err = storage_read(key, value);
  if (err) {
    return err;
  }
  pre_cache[key] = *value;
  return 0;
}

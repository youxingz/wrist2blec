#include <stdint.h>

typedef enum {
  PERSISTENCE_KEY_ENABLE = 0x0000,
  PERSISTENCE_KEY_YAW_THRESHOLD = 0x0001,
  PERSISTENCE_KEY_PITCH_THRESHOLD = 0x0002,
  PERSISTENCE_KEY_ROLL_THRESHOLD = 0x0003,
  PERSISTENCE_KEY_MOTOR_MODE = 0x0004,
  PERSISTENCE_KEY_HAND_MODE = 0x0005,
  PERSISTENCE_KEY_PRECISION_MODE = 0x0006,
  PERSISTENCE_KEY_VIBER_MODE = 0x0007,
  PERSISTENCE_KEY_LEN,
} presistence_key_t;

int presistence_init();

int presistence_upsert(presistence_key_t key, uint8_t value);

int presistence_get(presistence_key_t key, uint8_t *value);

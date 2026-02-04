#ifndef STORAGE_H__
#define STORAGE_H__

#include <stdint.h>

int storage_upsert(uint16_t key, uint8_t value);
int storage_read(uint16_t key, uint8_t *value);

#endif

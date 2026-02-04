#include "inc/storage.h"

#include <zephyr/device.h>
#include <zephyr/drivers/flash.h>
#include <zephyr/fs/nvs.h>
#include <zephyr/logging/log.h>
#include <zephyr/storage/flash_map.h>
#include <errno.h>

LOG_MODULE_REGISTER(storage, LOG_LEVEL_INF);

static struct nvs_fs aae_nvs;
static bool aae_nvs_ready;

static int storage_init(void)
{
	if (aae_nvs_ready) {
		return 0;
	}

	const struct flash_area *fa = NULL;
	struct flash_pages_info info;
	int err = flash_area_open(FIXED_PARTITION_ID(storage_partition), &fa);
	if (err) {
		LOG_INF("NVS open storage failed (err %d)", err);
		return err;
	}
	if (!flash_area_device_is_ready(fa)) {
		LOG_INF("NVS flash device not ready");
		flash_area_close(fa);
		return -ENODEV;
	}

	aae_nvs.flash_device = fa->fa_dev;
	aae_nvs.offset = fa->fa_off;

	err = flash_get_page_info_by_offs(aae_nvs.flash_device, aae_nvs.offset, &info);
	if (err) {
		LOG_INF("NVS page info failed (err %d)", err);
		flash_area_close(fa);
		return err;
	}

	aae_nvs.sector_size = info.size;
	aae_nvs.sector_count = fa->fa_size / info.size;
	flash_area_close(fa);

	err = nvs_mount(&aae_nvs);
	if (err) {
		LOG_INF("NVS init failed (err %d)", err);
		return err;
	}

	aae_nvs_ready = true;
	return 0;
}

int storage_upsert(uint16_t key, uint8_t value)
{
	int err = storage_init();
	if (err) {
		return err;
	}

	err = nvs_write(&aae_nvs, key, &value, sizeof(value));
	if (err < 0) {
		return err;
	}

	return 0;
}

#include "inc/ble_conn.h"

#include <zephyr/kernel.h>
#include <errno.h>
#include <zephyr/bluetooth/bluetooth.h>
#include <zephyr/bluetooth/hci.h>
#include <zephyr/bluetooth/conn.h>
#include <zephyr/bluetooth/gatt.h>
#include <zephyr/bluetooth/uuid.h>

#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(blec, LOG_LEVEL_INF);

#define DEVICE_NAME     CONFIG_BT_DEVICE_NAME
#define DEVICE_NAME_LEN (sizeof(DEVICE_NAME) - 1)

static volatile bool device_connected = false;

static void adv_restart_work_handler(struct k_work *work);
K_WORK_DELAYABLE_DEFINE(adv_restart_work, adv_restart_work_handler);

/* Advertising data (ADV) */
static const struct bt_data ad[] = {
	BT_DATA_BYTES(BT_DATA_FLAGS, (BT_LE_AD_GENERAL | BT_LE_AD_NO_BREDR)),

	BT_DATA_BYTES(BT_DATA_UUID16_ALL,
		      BT_UUID_16_ENCODE(0xAAAA),            /* AAAA */
		      BT_UUID_16_ENCODE(BT_UUID_BAS_VAL),   /* Battery Service */
		      BT_UUID_16_ENCODE(BT_UUID_DIS_VAL)),  /* Device Information Service */
};

/* Scan response data (SCAN_RSP) */
static const struct bt_data sd[] = {
	BT_DATA(BT_DATA_NAME_COMPLETE, DEVICE_NAME, DEVICE_NAME_LEN),
};

static void connected(struct bt_conn *conn, uint8_t conn_err)
{
	int err;
	struct bt_conn_info info;
	char addr[BT_ADDR_LE_STR_LEN];

	bt_addr_le_to_str(bt_conn_get_dst(conn), addr, sizeof(addr));

	if (conn_err) {
		printk("ble_conn: connect failed (err %u)\n", conn_err);
		LOG_INF("Connection failed (err %u)\n", conn_err);
		device_connected = false;
		/* 连接失败后，确保广播重新起来（延迟重试） */
		k_work_schedule(&adv_restart_work, K_MSEC(200));
		return;
	}

	if (!device_connected) {
		device_connected = true;
		/* stop adv */
		ble_adv_stop();
	}

	err = bt_conn_get_info(conn, &info);
	if (err) {
		LOG_INF("Failed to get connection info (err %d)\n", err);
		return;
	}

	if (info.type == BT_CONN_TYPE_LE) {
		const struct bt_conn_le_phy_info *phy_info = info.le.phy;

		LOG_INF("Connected: %s, tx_phy %u, rx_phy %u\n",
			addr, phy_info->tx_phy, phy_info->rx_phy);
		printk("ble_conn: connected %s, tx_phy %u, rx_phy %u\n",
		       addr, phy_info->tx_phy, phy_info->rx_phy);
	} else {
		LOG_INF("Connected: %s (non-LE?)\n", addr);
		printk("ble_conn: connected %s (non-LE?)\n", addr);
	}
}

static void disconnected(struct bt_conn *conn, uint8_t reason)
{
	LOG_INF("Disconnected (reason 0x%02x)\n", reason);
	printk("ble_conn: disconnected (reason 0x%02x)\n", reason);

	if (device_connected) {
		device_connected = false;
	}
	/* start adv again (delay to avoid -EAGAIN) */
	k_work_schedule(&adv_restart_work, K_MSEC(200));
}

/* Define connection callback */
BT_CONN_CB_DEFINE(conn_callbacks) = {
	.connected = connected,
	.disconnected = disconnected,
};

int ble_adv_start(void)
{
	/*
	 * 最新 Zephyr 推荐：显式构造参数
	 * - CONNECTABLE：可连接
	 * - 使用 FAST_INT_2：常见的快速广播间隔
	 * - 这里不加 BT_LE_ADV_OPT_USE_NAME，因为我们把 NAME 放在 sd[] 里了
	 */
	const struct bt_le_adv_param *adv_param =
		BT_LE_ADV_PARAM(BT_LE_ADV_OPT_SCANNABLE | BT_LE_ADV_OPT_CONN | BT_LE_ADV_OPT_USE_IDENTITY,
				BT_GAP_ADV_FAST_INT_MIN_2,
				BT_GAP_ADV_FAST_INT_MAX_2,
				NULL);

	int err = bt_le_adv_start(adv_param,
			      ad, ARRAY_SIZE(ad),
			      sd, ARRAY_SIZE(sd));
	if (err) {
		LOG_INF("Failed to start advertising (err %d)\n", err);
		return err;
	}

	LOG_INF("Advertiser started\n");
	return 0;
}

int ble_adv_stop(void)
{
	return bt_le_adv_stop();
}

int ble_adv_init(void)
{
	/* 你这里目前没做额外初始化，保持原样 */
	return 0;
}

static void adv_restart_work_handler(struct k_work *work)
{
	int err = ble_adv_start();

	if (err == -EALREADY) {
		LOG_INF("Advertiser already active\n");
		printk("ble_conn: adv already active\n");
		return;
	}

	if (err) {
		LOG_INF("Advertiser restart failed (err %d), retry\n", err);
		printk("ble_conn: adv restart failed (err %d), retry\n", err);
		k_work_schedule(&adv_restart_work, K_MSEC(200));
	}
}

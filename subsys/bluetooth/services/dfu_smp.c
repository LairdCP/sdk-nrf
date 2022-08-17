/*
 * Copyright (c) 2019 Nordic Semiconductor ASA
 *
 * SPDX-License-Identifier: LicenseRef-Nordic-5-Clause
 */
#include <zephyr/kernel.h>
#include <zephyr/bluetooth/conn.h>
#include <zephyr/bluetooth/uuid.h>
#include <zephyr/bluetooth/gatt.h>

#include <bluetooth/services/dfu_smp.h>

#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(dfu_smp, CONFIG_BT_DFU_SMP_LOG_LEVEL);

/* Number of bytes of overhead in a GATT write (bytes taken from MTU size) */
#define GATT_WRITE_OVERHEAD 3


/** @brief Notification callback function
 *
 *  Internal function used to process the response from the SMP characteristic.
 *
 *  @param conn   Connection handler.
 *  @param params Notification parameters structure - the pointer
 *                to the structure provided to subscribe function.
 *  @param data   Pointer to the data buffer.
 *  @param length The size of the received data.
 *
 *  @retval BT_GATT_ITER_STOP     Stop notification
 *  @retval BT_GATT_ITER_CONTINUE Continue notification
 */
static uint8_t notify_process(struct bt_conn *conn,
			      struct bt_gatt_subscribe_params *params,
			      const void *data, uint16_t length)
{
	struct bt_dfu_smp *dfu_smp;

	dfu_smp = CONTAINER_OF(params,
				 struct bt_dfu_smp,
				 notification_params);
	if (!data) {
		/* Notification disabled */
		dfu_smp->cbs.rsp_part = NULL;
		params->notify = NULL;
		return BT_GATT_ITER_STOP;
	}

	if (dfu_smp->cbs.rsp_part || dfu_smp->cbs.notif_cb) {
		dfu_smp->rsp_state.chunk_size = length;
		dfu_smp->rsp_state.data       = data;
		if (dfu_smp->rsp_state.offset == 0) {
			/* First block */
			uint32_t total_len;
			const struct bt_dfu_smp_header *header;

			header = data;
			total_len = (((uint16_t)header->len_h8) << 8) |
				    header->len_l8;
			total_len += sizeof(struct bt_dfu_smp_header);
			dfu_smp->rsp_state.total_size = total_len;
		}

		if (dfu_smp->cbs.rsp_part) {
			dfu_smp->cbs.rsp_part(dfu_smp);
		} else if (dfu_smp->cbs.notif_cb) {
			dfu_smp->cbs.notif_cb(dfu_smp);
		}

		dfu_smp->rsp_state.offset += length;
		if (dfu_smp->rsp_state.offset >=
			dfu_smp->rsp_state.total_size) {
			/* Whole response has been received */
			dfu_smp->cbs.rsp_part = NULL;
			memset(&(dfu_smp->rsp_state), 0, sizeof(dfu_smp->rsp_state));
		}


	} else {
		dfu_smp->cbs.error_cb(dfu_smp, -EIO);
	}
	return BT_GATT_ITER_CONTINUE;
}


int bt_dfu_smp_init(struct bt_dfu_smp *dfu_smp,
			   const struct bt_dfu_smp_init_params *params)
{
	if (!params->error_cb) {
		return -EINVAL;
	}
	memset(dfu_smp, 0, sizeof(*dfu_smp));
	dfu_smp->cbs.error_cb = params->error_cb;
	dfu_smp->cbs.notif_cb = params->notif_cb;
	return 0;
}


int bt_dfu_smp_handles_assign(struct bt_gatt_dm *dm,
			      struct bt_dfu_smp *dfu_smp)
{
	const struct bt_gatt_dm_attr *gatt_service_attr =
			bt_gatt_dm_service_get(dm);
	const struct bt_gatt_service_val *gatt_service =
			bt_gatt_dm_attr_service_val(gatt_service_attr);
	const struct bt_gatt_dm_attr *gatt_chrc;
	const struct bt_gatt_dm_attr *gatt_desc;

	if (bt_uuid_cmp(gatt_service->uuid, BT_UUID_DFU_SMP_SERVICE)) {
		return -ENOTSUP;
	}

	LOG_DBG("Getting handles from DFU SMP service.");

	/* Characteristic discovery */
	gatt_chrc = bt_gatt_dm_char_by_uuid(dm, BT_UUID_DFU_SMP_CHAR);
	if (!gatt_chrc) {
		LOG_ERR("No SMP characteristic found.");
		return -EINVAL;
	}
	gatt_desc = bt_gatt_dm_desc_by_uuid(dm, gatt_chrc,
					    BT_UUID_DFU_SMP_CHAR);
	if (!gatt_desc) {
		LOG_ERR("No characteristic value found.");
		return -EINVAL;
	}
	dfu_smp->handles.smp = gatt_desc->handle;
	gatt_desc = bt_gatt_dm_desc_by_uuid(dm, gatt_chrc, BT_UUID_GATT_CCC);
	if (!gatt_desc) {
		LOG_ERR("Missing characteristic CCC.");
		return -EINVAL;
	}
	dfu_smp->handles.smp_ccc = gatt_desc->handle;

	/* Save connection object */
	dfu_smp->conn = bt_gatt_dm_conn_get(dm);

	return 0;
}


int bt_dfu_smp_command(struct bt_dfu_smp *dfu_smp,
			      bt_dfu_smp_rsp_part_cb rsp_cb,
			      size_t cmd_size,
			      const void *cmd_data)
{
	uint16_t mtu;
	int ret;

	if (!dfu_smp || !rsp_cb || !cmd_data || cmd_size == 0) {
		return -EINVAL;
	}
	if (!dfu_smp->conn) {
		return -ENXIO;
	}

	mtu = bt_gatt_get_mtu(dfu_smp->conn);
	if (dfu_smp->cbs.rsp_part) {
		return -EBUSY;
	}
	/* Sign into notification if not currently enabled */
	if (!dfu_smp->notification_params.notify) {
		dfu_smp->notification_params.value_handle =
			dfu_smp->handles.smp;
		dfu_smp->notification_params.ccc_handle =
			dfu_smp->handles.smp_ccc;
		dfu_smp->notification_params.notify =
			notify_process;
		dfu_smp->notification_params.value  = BT_GATT_CCC_NOTIFY;
		atomic_set_bit(dfu_smp->notification_params.flags,
			       BT_GATT_SUBSCRIBE_FLAG_VOLATILE);

		ret = bt_gatt_subscribe(dfu_smp->conn,
					&dfu_smp->notification_params);
		if (ret) {
			return ret;
		}
	}
	memset(&dfu_smp->rsp_state, 0, sizeof(dfu_smp->rsp_state));
	dfu_smp->cbs.rsp_part = rsp_cb;

	/* Send chunks that fit into the MTU */
	while (cmd_size > 0) {
		size_t this_size = cmd_size;
		if (this_size > (mtu - GATT_WRITE_OVERHEAD)) {
			this_size = mtu - GATT_WRITE_OVERHEAD;
		}
		ret = bt_gatt_write_without_response(dfu_smp->conn,
			dfu_smp->handles.smp,
			cmd_data, this_size, false);
		if (ret) {
			dfu_smp->cbs.rsp_part = NULL;
			break;
		}
		cmd_size -= this_size;
		cmd_data = ((char *)cmd_data) + this_size;
	}

	return ret;
}

struct bt_conn *bt_dfu_smp_conn(
		const struct bt_dfu_smp *dfu_smp)
{
	return dfu_smp->conn;
}


const struct bt_dfu_smp_rsp_state *bt_dfu_smp_rsp_state(
		const struct bt_dfu_smp *dfu_smp)
{
	return &(dfu_smp->rsp_state);
}


bool bt_dfu_smp_rsp_total_check(
		const struct bt_dfu_smp *dfu_smp)
{
	return (dfu_smp->rsp_state.chunk_size + dfu_smp->rsp_state.offset
		>=
		dfu_smp->rsp_state.total_size);
}

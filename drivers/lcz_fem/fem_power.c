/*
 * Copyright (c) 2023 Laird Connectivity
 *
 * SPDX-License-Identifier: LicenseRef-LairdConnectivity-Clause
 */

#include <zephyr/kernel.h>
#include <zephyr/init.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(lcz_fem_power, CONFIG_LCZ_FEM_LOG_LEVEL);

#include <mpsl.h>
#include <mpsl_tx_power.h>
#include <mpsl_fem_protocol_api.h>

#include "fem_power_table.h"

#if defined(CONFIG_MPSL_FEM_NRF21540_GPIO_SPI_SUPPORT) || defined(CONFIG_MPSL_FEM_POWER_MODEL) ||  \
	defined(CONFIG_MPSL_FEM_NRF21540_RUNTIME_PA_GAIN_CONTROL)
#error "Unsupported MPSL FEM configuration"
#endif

/* Mode pin is grounded on BL5340PA */
#if CONFIG_MPSL_FEM_NRF21540_TX_GAIN_DB != 20
#error "TX Gain must be set to 20"
#endif

static int lcz_fem_power_init(const struct device *dev)
{
	ARG_UNUSED(dev);
	int r = 0;

	LOG_INF("%s", POWER_TABLE_STR);

	do {
		if (!mpsl_is_initialized()) {
			LOG_ERR("%s: MPSL is NOT initialized", __func__);
			r = -EPERM;
			break;
		}

#if defined(CONFIG_BT_CTLR_PHY_CODED)
		/* Use power tables based on antenna type and region */
		r = mpsl_tx_power_channel_map_set(&ENV_125K);
		if (r < 0) {
			LOG_ERR("Unable to set 125K power envelope");
			break;
		}

		r = mpsl_tx_power_channel_map_set(&ENV_500K);
		if (r < 0) {
			LOG_ERR("Unable to set 500K power envelope");
			break;
		}
#endif

#if defined(CONFIG_BT)
		r = mpsl_tx_power_channel_map_set(&ENV_1M);
		if (r < 0) {
			LOG_ERR("Unable to set 1M power envelope");
			break;
		}

		r = mpsl_tx_power_channel_map_set(&ENV_2M);
		if (r < 0) {
			LOG_ERR("Unable to set 2M power envelope");
			break;
		}
#endif

#if defined(CONFIG_NRF_802154_SL)
		r = mpsl_tx_power_channel_map_set(&ENV_802154);
		if (r < 0) {
			LOG_ERR("Unable to set 802.15.4 power envelope");
			break;
		}
#endif

	} while (0);

	LOG_INF("%s status: %d", __func__, r);

	return r;
}

SYS_INIT(lcz_fem_power_init, APPLICATION, CONFIG_APPLICATION_INIT_PRIORITY);

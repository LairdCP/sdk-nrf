/*
 * Copyright (c) 2023 Laird Connectivity
 *
 * SPDX-License-Identifier: LicenseRef-LairdConnectivity-Clause
 */

#include <zephyr/kernel.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(lcz_fem_power_model_ce, CONFIG_LCZ_FEM_LOG_LEVEL);

#include <mpsl.h>
#include <mpsl_tx_power.h>
#include <mpsl_fem_protocol_api.h>
#include <mpsl_fem_power_model.h>

/* Value of nRF5340 output (SoC) with 18 dB of gain at the FEM to achieve CE compliance. */
#define SOC_POWER (-16)

/* The gain of the FEM is always set to 23 regardless of the 20 dB calibration point.
 * The same value is used for all voltages, temperatures, channels, and modulations.
 */
static mpsl_fem_power_model_output_t flat = {
	.achieved_pwr = 4,
	.soc_pwr = SOC_POWER,
	.fem.gain_db = 18,
	.fem.private_setting = 23,
};

/* For debug purposes, print out the calibration values.
 * On nRF5340, RTT or USB must be used for the console because SPI cannot be used with uart0.
 */
static void log_calibration_values(const mpsl_fem_calibration_data_t *p_calibration_data)
{
	mpsl_fem_gain_t gain;

	gain = p_calibration_data->nrf21540_gpio_spi.pouta;
	LOG_INF("Calibration A: gain dB: %d private: %u", gain.gain_db, gain.private_setting);

	gain = p_calibration_data->nrf21540_gpio_spi.poutb;
	LOG_INF("Calibration B: gain dB: %d private: %u", gain.gain_db, gain.private_setting);
}

static void laird_ce_model_init(const mpsl_fem_calibration_data_t *p_calibration_data)
{
	LOG_INF("CE Power Model");

	if (mpsl_tx_power_radio_supported_power_adjust(SOC_POWER, false) != SOC_POWER) {
		LOG_ERR("Unsupported SoC output power requested");
	}

	log_calibration_values(p_calibration_data);
}

static void laird_ce_model_fetch(int8_t requested_power, uint16_t freq_mhz,
				 mpsl_fem_power_model_output_t *p_output, bool tx_power_ceiling)
{
	*p_output = flat;
}

/* Use the Laird 'model' instead of the Nordic model. */
const mpsl_fem_power_model_t *mpsl_fem_power_model_to_use_get(void)
{
	static const mpsl_fem_power_model_t LAIRD_CE_MODEL = {
		.fetch = laird_ce_model_fetch,
		.init = laird_ce_model_init,
	};

	return &LAIRD_CE_MODEL;
}

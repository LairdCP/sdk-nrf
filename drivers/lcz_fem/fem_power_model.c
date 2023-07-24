/*
 * Copyright (c) 2023 Laird Connectivity
 *
 * Power Model that handles Feb2023 Errata.
 *
 * SPDX-License-Identifier: LicenseRef-LairdConnectivity-Clause
 */

#include <zephyr/kernel.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(lcz_fem_power_model, CONFIG_LCZ_FEM_LOG_LEVEL);

#include <mpsl.h>
#include <mpsl_tx_power.h>
#include <mpsl_fem_protocol_api.h>
#include <mpsl_fem_power_model.h>
#include <mpsl_temp.h>

#define LCZ_FEM_POWER_REQ_MIN -40
#define LCZ_FEM_POWER_REQ_MAX 20
#define LCZ_FEM_GAIN 20

#define MIN_FEM_SETTING 18
#define ERRATA_FEM_SETTING 23

static mpsl_fem_power_model_output_t table[61];

/* Build power table for current processor and FEM.
 * Table must be used because fetch must occur in 2 us or less.
 */
static void build_power_table(const mpsl_fem_calibration_data_t *p_calibration_data)
{
	size_t i;
	mpsl_tx_power_t req = LCZ_FEM_POWER_REQ_MIN;
	mpsl_tx_power_t soc_desired;
	mpsl_tx_power_t soc_actual;
	mpsl_fem_power_model_output_t *p;
	uint8_t fem_setting;

	/* Enter a safe value for the gain to override boards that were
	 * incorrectly programmed during production of Feb 2023.
	 */
	fem_setting = p_calibration_data->nrf21540_gpio_spi.pouta.private_setting;
	if (fem_setting < MIN_FEM_SETTING) {
		LOG_WRN("Overriding FEM setting of %u with %u", fem_setting, ERRATA_FEM_SETTING);
		fem_setting = ERRATA_FEM_SETTING;
	}

	/* Generate split between SoC and FEM.
	 * Pretend gain is constant for every intput.
	 */
	for (i = 0; i < ARRAY_SIZE(table); i++) {
		p = table + i;
		soc_desired = req - LCZ_FEM_GAIN;
		soc_actual = mpsl_tx_power_radio_supported_power_adjust(soc_desired, false);

		p->achieved_pwr = soc_actual + LCZ_FEM_GAIN;
		p->soc_pwr = soc_actual;
		p->fem.gain_db = LCZ_FEM_GAIN;
		p->fem.private_setting = fem_setting;
		if (IS_ENABLED(CONFIG_LCZ_FEM_LOG_POWER_MODEL)) {
			LOG_DBG("%2u req: %3d actual: %3d radio: %3d ", i, req, p->achieved_pwr,
				p->soc_pwr);
		}
		req += 1;
	}
}

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

static void laird_model_init(const mpsl_fem_calibration_data_t *p_calibration_data)
{
	LOG_INF("Generic Power Model");

	log_calibration_values(p_calibration_data);
	build_power_table(p_calibration_data);
}

static void laird_model_fetch(int8_t requested_power, uint16_t freq_mhz,
			      mpsl_fem_power_model_output_t *p_output, bool tx_power_ceiling)
{
	int8_t offset;

	/* Clamp between -40 and 20. Generate table index. */
	offset = requested_power - LCZ_FEM_POWER_REQ_MIN;
	if (offset < 0) {
		offset = 0;
	} else if (offset > ARRAY_SIZE(table)) {
		offset = ARRAY_SIZE(table) - 1;
	}

	*p_output = table[offset];
}

/* Use the Laird model instead of the Nordic model. */
const mpsl_fem_power_model_t *mpsl_fem_power_model_to_use_get(void)
{
	static const mpsl_fem_power_model_t LAIRD_MODEL = {
		.fetch = laird_model_fetch,
		.init = laird_model_init,
	};

	return &LAIRD_MODEL;
}

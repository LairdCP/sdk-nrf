/*
 * Copyright (c) 2023 Laird Connectivity
 *
 * When using RTT with network CPU, connect using viewer and then
 * reset from application shell.
 *
 * SPDX-License-Identifier: LicenseRef-LairdConnectivity-Clause
 */

#include <zephyr/kernel.h>
#include <zephyr/init.h>
#include <zephyr/timing/timing.h>
#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(lcz_fem_debug, CONFIG_LCZ_FEM_LOG_LEVEL);

#include <mpsl.h>
#include <mpsl_tx_power.h>
#include <mpsl_fem_protocol_api.h>
#include <mpsl_fem_power_model.h>

const mpsl_tx_power_t REQUEST[] = {
	30, 20, 18, 16, 15, 14, 12, 11, 10, 9,	8,   7,	  6,   5,   4,	 3,   2,   1,
	0,  -1, -2, -3, -4, -5, -6, -7, -8, -9, -10, -11, -12, -16, -20, -30, -40, -50,
};

extern const mpsl_fem_power_model_t *mpsl_fem_power_model_to_use_get(void);

static void log_power_adjust_table(void)
{
	size_t i;

	LOG_DBG("Requested SoC Power -> Actual");

	for (i = 0; i < sizeof(REQUEST); i++) {
		LOG_DBG("%3d -> %3d", REQUEST[i],
			mpsl_tx_power_radio_supported_power_adjust(REQUEST[i], false));
	}
}

static void log_power_split(void)
{
	const uint16_t FREQ = 2402;
	const bool CEILING = false;
	mpsl_tx_power_split_t split = { 0 };
	mpsl_tx_power_t req;
	mpsl_tx_power_t actual;
	size_t i;

	LOG_DBG("Antenna Power");

	/*
     * Private setting will be blank if GPIO only mode is used.
     * Power model allows private setting to be changed.
     * Runtime PA gain control allows switching between 20/10.
     */
	for (i = 0; i < ARRAY_SIZE(REQUEST); i++) {
		req = REQUEST[i];
		actual = mpsl_fem_tx_power_split(req, &split, FREQ, CEILING);
		LOG_DBG("req: %3d actual: %3d radio: %3d gain: %2d private: %2u", req, actual,
			split.radio_tx_power, split.fem.gain_db, split.fem.private_setting);
	}
}

static void log_model_timing(void)
{
	const uint16_t FREQ = 2402;
	mpsl_tx_power_t req;
	timing_t start_time;
	timing_t end_time;
	uint64_t total_cycles;
	uint64_t total_ns;
	mpsl_fem_power_model_output_t output;
	const mpsl_fem_power_model_t *model = mpsl_fem_power_model_to_use_get();
	int key;
	size_t i;

	LOG_DBG("Model timing (2 us max)");

	/*
     * Private setting will be blank if GPIO only mode is used.
     * Power model allows private setting to be changed.
     * Runtime PA gain control allows switching between 20/10.
     */
	for (i = 0; i < ARRAY_SIZE(REQUEST); i++) {
		req = REQUEST[i];
		key = irq_lock();
		start_time = timing_counter_get();
		model->fetch(req, FREQ, &output, false);
		end_time = timing_counter_get();
		irq_unlock(key);
		total_cycles = timing_cycles_get(&start_time, &end_time);
		total_ns = timing_cycles_to_ns(total_cycles);
		LOG_DBG("req: %3d actual: %3d radio: %3d gain: %2d private: %2u compute_time: %llu ns",
			req, output.achieved_pwr, output.soc_pwr, output.fem.gain_db,
			output.fem.private_setting, total_ns);
	}
}

static int lcz_fem_debug_init(void))
{
	if (IS_ENABLED(CONFIG_LCZ_FEM_DEBUG_POWER_ADJUST)) {
		log_power_adjust_table();
	}

	if (IS_ENABLED(CONFIG_LCZ_FEM_DEBUG_POWER_SPLIT)) {
		log_power_split();
	}

	if (IS_ENABLED(CONFIG_LCZ_FEM_DEBUG_MODEL_TIMING)) {
		timing_init();
		timing_start();
		log_model_timing();
		timing_stop();
	}

	return 0;
}

SYS_INIT(lcz_fem_debug_init, APPLICATION, 99);

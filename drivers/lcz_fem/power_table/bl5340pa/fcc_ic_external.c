/*
 * Copyright (c) 2023-2024 Ezurio
 *
 * SPDX-License-Identifier: LicenseRef-Ezurio-Clause
 */

#include <mpsl_tx_power.h>

#include "fem_power_table.h"

const char * const POWER_TABLE_STR = "BL5340PA Region: FCC/IC Antenna Type: EXTERNAL";

#if defined(CONFIG_BT_CTLR_PHY_CODED)
const mpsl_tx_power_envelope_t ENV_125K = {
	.phy = MPSL_PHY_BLE_LR125Kbit,
	.envelope.tx_power_ble[0]  = MPSL_FEM_POWER_REDUCE(8),
	.envelope.tx_power_ble[1]  = MPSL_FEM_POWER_REDUCE(8),
	.envelope.tx_power_ble[2]  = MPSL_FEM_POWER_REDUCE(8),
	.envelope.tx_power_ble[3]  = MPSL_FEM_POWER_REDUCE(8),
	.envelope.tx_power_ble[4]  = MPSL_FEM_POWER_REDUCE(8),
	.envelope.tx_power_ble[5]  = MPSL_FEM_POWER_REDUCE(8),
	.envelope.tx_power_ble[6]  = MPSL_FEM_POWER_REDUCE(8),
	.envelope.tx_power_ble[7]  = MPSL_FEM_POWER_REDUCE(8),
	.envelope.tx_power_ble[8]  = MPSL_FEM_POWER_REDUCE(8),
	.envelope.tx_power_ble[9]  = MPSL_FEM_POWER_REDUCE(8),
	.envelope.tx_power_ble[10] = MPSL_FEM_POWER_REDUCE(8),
	.envelope.tx_power_ble[11] = MPSL_FEM_POWER_REDUCE(8),
	.envelope.tx_power_ble[12] = MPSL_FEM_POWER_REDUCE(8),
	.envelope.tx_power_ble[13] = MPSL_FEM_POWER_REDUCE(8),
	.envelope.tx_power_ble[14] = MPSL_FEM_POWER_REDUCE(8),
	.envelope.tx_power_ble[15] = MPSL_FEM_POWER_REDUCE(8),
	.envelope.tx_power_ble[16] = MPSL_FEM_POWER_REDUCE(8),
	.envelope.tx_power_ble[17] = MPSL_FEM_POWER_REDUCE(8),
	.envelope.tx_power_ble[18] = MPSL_FEM_POWER_REDUCE(8),
	.envelope.tx_power_ble[19] = MPSL_FEM_POWER_REDUCE(8),
	.envelope.tx_power_ble[20] = MPSL_FEM_POWER_REDUCE(8),
	.envelope.tx_power_ble[21] = MPSL_FEM_POWER_REDUCE(8),
	.envelope.tx_power_ble[22] = MPSL_FEM_POWER_REDUCE(8),
	.envelope.tx_power_ble[23] = MPSL_FEM_POWER_REDUCE(8),
	.envelope.tx_power_ble[24] = MPSL_FEM_POWER_REDUCE(8),
	.envelope.tx_power_ble[25] = MPSL_FEM_POWER_REDUCE(8),
	.envelope.tx_power_ble[26] = MPSL_FEM_POWER_REDUCE(8),
	.envelope.tx_power_ble[27] = MPSL_FEM_POWER_REDUCE(8),
	.envelope.tx_power_ble[28] = MPSL_FEM_POWER_REDUCE(8),
	.envelope.tx_power_ble[29] = MPSL_FEM_POWER_REDUCE(8),
	.envelope.tx_power_ble[30] = MPSL_FEM_POWER_REDUCE(8),
	.envelope.tx_power_ble[31] = MPSL_FEM_POWER_REDUCE(8),
	.envelope.tx_power_ble[32] = MPSL_FEM_POWER_REDUCE(8),
	.envelope.tx_power_ble[33] = MPSL_FEM_POWER_REDUCE(8),
	.envelope.tx_power_ble[34] = MPSL_FEM_POWER_REDUCE(8),
	.envelope.tx_power_ble[35] = MPSL_FEM_POWER_REDUCE(8),
	.envelope.tx_power_ble[36] = MPSL_FEM_POWER_REDUCE(8),
	.envelope.tx_power_ble[37] = MPSL_FEM_POWER_REDUCE(8),
	.envelope.tx_power_ble[38] = MPSL_FEM_POWER_REDUCE(8),
	.envelope.tx_power_ble[39] = MPSL_FEM_POWER_REDUCE(8),
};

const mpsl_tx_power_envelope_t ENV_500K = {
	.phy = MPSL_PHY_BLE_LR500Kbit,
	.envelope.tx_power_ble[0]  = MPSL_FEM_POWER_REDUCE(5),
	.envelope.tx_power_ble[1]  = MPSL_FEM_POWER_REDUCE(5),
	.envelope.tx_power_ble[2]  = MPSL_FEM_POWER_REDUCE(5),
	.envelope.tx_power_ble[3]  = MPSL_FEM_POWER_REDUCE(5),
	.envelope.tx_power_ble[4]  = MPSL_FEM_POWER_REDUCE(5),
	.envelope.tx_power_ble[5]  = MPSL_FEM_POWER_REDUCE(5),
	.envelope.tx_power_ble[6]  = MPSL_FEM_POWER_REDUCE(5),
	.envelope.tx_power_ble[7]  = MPSL_FEM_POWER_REDUCE(5),
	.envelope.tx_power_ble[8]  = MPSL_FEM_POWER_REDUCE(5),
	.envelope.tx_power_ble[9]  = MPSL_FEM_POWER_REDUCE(5),
	.envelope.tx_power_ble[10] = MPSL_FEM_POWER_REDUCE(5),
	.envelope.tx_power_ble[11] = MPSL_FEM_POWER_REDUCE(5),
	.envelope.tx_power_ble[12] = MPSL_FEM_POWER_REDUCE(5),
	.envelope.tx_power_ble[13] = MPSL_FEM_POWER_REDUCE(5),
	.envelope.tx_power_ble[14] = MPSL_FEM_POWER_REDUCE(5),
	.envelope.tx_power_ble[15] = MPSL_FEM_POWER_REDUCE(5),
	.envelope.tx_power_ble[16] = MPSL_FEM_POWER_REDUCE(5),
	.envelope.tx_power_ble[17] = MPSL_FEM_POWER_REDUCE(6),
	.envelope.tx_power_ble[18] = MPSL_FEM_POWER_REDUCE(6),
	.envelope.tx_power_ble[19] = MPSL_FEM_POWER_REDUCE(6),
	.envelope.tx_power_ble[20] = MPSL_FEM_POWER_REDUCE(6),
	.envelope.tx_power_ble[21] = MPSL_FEM_POWER_REDUCE(6),
	.envelope.tx_power_ble[22] = MPSL_FEM_POWER_REDUCE(6),
	.envelope.tx_power_ble[23] = MPSL_FEM_POWER_REDUCE(6),
	.envelope.tx_power_ble[24] = MPSL_FEM_POWER_REDUCE(6),
	.envelope.tx_power_ble[25] = MPSL_FEM_POWER_REDUCE(6),
	.envelope.tx_power_ble[26] = MPSL_FEM_POWER_REDUCE(6),
	.envelope.tx_power_ble[27] = MPSL_FEM_POWER_REDUCE(6),
	.envelope.tx_power_ble[28] = MPSL_FEM_POWER_REDUCE(6),
	.envelope.tx_power_ble[29] = MPSL_FEM_POWER_REDUCE(6),
	.envelope.tx_power_ble[30] = MPSL_FEM_POWER_REDUCE(6),
	.envelope.tx_power_ble[31] = MPSL_FEM_POWER_REDUCE(6),
	.envelope.tx_power_ble[32] = MPSL_FEM_POWER_REDUCE(6),
	.envelope.tx_power_ble[33] = MPSL_FEM_POWER_REDUCE(6),
	.envelope.tx_power_ble[34] = MPSL_FEM_POWER_REDUCE(6),
	.envelope.tx_power_ble[35] = MPSL_FEM_POWER_REDUCE(6),
	.envelope.tx_power_ble[36] = MPSL_FEM_POWER_REDUCE(6),
	.envelope.tx_power_ble[37] = MPSL_FEM_POWER_REDUCE(5),
	.envelope.tx_power_ble[38] = MPSL_FEM_POWER_REDUCE(5),
	.envelope.tx_power_ble[39] = MPSL_FEM_POWER_REDUCE(7),
};
#endif

#if defined(CONFIG_BT)
const mpsl_tx_power_envelope_t ENV_1M = {
	.phy = MPSL_PHY_BLE_1M,
	.envelope.tx_power_ble[0]  = MPSL_FEM_POWER_REDUCE(5),
	.envelope.tx_power_ble[1]  = MPSL_FEM_POWER_REDUCE(5),
	.envelope.tx_power_ble[2]  = MPSL_FEM_POWER_REDUCE(5),
	.envelope.tx_power_ble[3]  = MPSL_FEM_POWER_REDUCE(5),
	.envelope.tx_power_ble[4]  = MPSL_FEM_POWER_REDUCE(5),
	.envelope.tx_power_ble[5]  = MPSL_FEM_POWER_REDUCE(5),
	.envelope.tx_power_ble[6]  = MPSL_FEM_POWER_REDUCE(5),
	.envelope.tx_power_ble[7]  = MPSL_FEM_POWER_REDUCE(5),
	.envelope.tx_power_ble[8]  = MPSL_FEM_POWER_REDUCE(5),
	.envelope.tx_power_ble[9]  = MPSL_FEM_POWER_REDUCE(5),
	.envelope.tx_power_ble[10] = MPSL_FEM_POWER_REDUCE(5),
	.envelope.tx_power_ble[11] = MPSL_FEM_POWER_REDUCE(5),
	.envelope.tx_power_ble[12] = MPSL_FEM_POWER_REDUCE(5),
	.envelope.tx_power_ble[13] = MPSL_FEM_POWER_REDUCE(5),
	.envelope.tx_power_ble[14] = MPSL_FEM_POWER_REDUCE(5),
	.envelope.tx_power_ble[15] = MPSL_FEM_POWER_REDUCE(5),
	.envelope.tx_power_ble[16] = MPSL_FEM_POWER_REDUCE(5),
	.envelope.tx_power_ble[17] = MPSL_FEM_POWER_REDUCE(6),
	.envelope.tx_power_ble[18] = MPSL_FEM_POWER_REDUCE(6),
	.envelope.tx_power_ble[19] = MPSL_FEM_POWER_REDUCE(6),
	.envelope.tx_power_ble[20] = MPSL_FEM_POWER_REDUCE(6),
	.envelope.tx_power_ble[21] = MPSL_FEM_POWER_REDUCE(6),
	.envelope.tx_power_ble[22] = MPSL_FEM_POWER_REDUCE(6),
	.envelope.tx_power_ble[23] = MPSL_FEM_POWER_REDUCE(6),
	.envelope.tx_power_ble[24] = MPSL_FEM_POWER_REDUCE(6),
	.envelope.tx_power_ble[25] = MPSL_FEM_POWER_REDUCE(6),
	.envelope.tx_power_ble[26] = MPSL_FEM_POWER_REDUCE(6),
	.envelope.tx_power_ble[27] = MPSL_FEM_POWER_REDUCE(6),
	.envelope.tx_power_ble[28] = MPSL_FEM_POWER_REDUCE(6),
	.envelope.tx_power_ble[29] = MPSL_FEM_POWER_REDUCE(6),
	.envelope.tx_power_ble[30] = MPSL_FEM_POWER_REDUCE(6),
	.envelope.tx_power_ble[31] = MPSL_FEM_POWER_REDUCE(6),
	.envelope.tx_power_ble[32] = MPSL_FEM_POWER_REDUCE(6),
	.envelope.tx_power_ble[33] = MPSL_FEM_POWER_REDUCE(6),
	.envelope.tx_power_ble[34] = MPSL_FEM_POWER_REDUCE(6),
	.envelope.tx_power_ble[35] = MPSL_FEM_POWER_REDUCE(6),
	.envelope.tx_power_ble[36] = MPSL_FEM_POWER_REDUCE(6),
	.envelope.tx_power_ble[37] = MPSL_FEM_POWER_REDUCE(5),
	.envelope.tx_power_ble[38] = MPSL_FEM_POWER_REDUCE(5),
	.envelope.tx_power_ble[39] = MPSL_FEM_POWER_REDUCE(7),
};

const mpsl_tx_power_envelope_t ENV_2M = {
	.phy = MPSL_PHY_BLE_2M,
	.envelope.tx_power_ble[0]  = MPSL_FEM_POWER_REDUCE(5),
	.envelope.tx_power_ble[1]  = MPSL_FEM_POWER_REDUCE(5),
	.envelope.tx_power_ble[2]  = MPSL_FEM_POWER_REDUCE(5),
	.envelope.tx_power_ble[3]  = MPSL_FEM_POWER_REDUCE(5),
	.envelope.tx_power_ble[4]  = MPSL_FEM_POWER_REDUCE(5),
	.envelope.tx_power_ble[5]  = MPSL_FEM_POWER_REDUCE(5),
	.envelope.tx_power_ble[6]  = MPSL_FEM_POWER_REDUCE(5),
	.envelope.tx_power_ble[7]  = MPSL_FEM_POWER_REDUCE(5),
	.envelope.tx_power_ble[8]  = MPSL_FEM_POWER_REDUCE(5),
	.envelope.tx_power_ble[9]  = MPSL_FEM_POWER_REDUCE(5),
	.envelope.tx_power_ble[10] = MPSL_FEM_POWER_REDUCE(5),
	.envelope.tx_power_ble[11] = MPSL_FEM_POWER_REDUCE(5),
	.envelope.tx_power_ble[12] = MPSL_FEM_POWER_REDUCE(5),
	.envelope.tx_power_ble[13] = MPSL_FEM_POWER_REDUCE(5),
	.envelope.tx_power_ble[14] = MPSL_FEM_POWER_REDUCE(5),
	.envelope.tx_power_ble[15] = MPSL_FEM_POWER_REDUCE(5),
	.envelope.tx_power_ble[16] = MPSL_FEM_POWER_REDUCE(5),
	.envelope.tx_power_ble[17] = MPSL_FEM_POWER_REDUCE(6),
	.envelope.tx_power_ble[18] = MPSL_FEM_POWER_REDUCE(6),
	.envelope.tx_power_ble[19] = MPSL_FEM_POWER_REDUCE(6),
	.envelope.tx_power_ble[20] = MPSL_FEM_POWER_REDUCE(6),
	.envelope.tx_power_ble[21] = MPSL_FEM_POWER_REDUCE(6),
	.envelope.tx_power_ble[22] = MPSL_FEM_POWER_REDUCE(6),
	.envelope.tx_power_ble[23] = MPSL_FEM_POWER_REDUCE(6),
	.envelope.tx_power_ble[24] = MPSL_FEM_POWER_REDUCE(6),
	.envelope.tx_power_ble[25] = MPSL_FEM_POWER_REDUCE(6),
	.envelope.tx_power_ble[26] = MPSL_FEM_POWER_REDUCE(6),
	.envelope.tx_power_ble[27] = MPSL_FEM_POWER_REDUCE(6),
	.envelope.tx_power_ble[28] = MPSL_FEM_POWER_REDUCE(6),
	.envelope.tx_power_ble[29] = MPSL_FEM_POWER_REDUCE(6),
	.envelope.tx_power_ble[30] = MPSL_FEM_POWER_REDUCE(6),
	.envelope.tx_power_ble[31] = MPSL_FEM_POWER_REDUCE(6),
	.envelope.tx_power_ble[32] = MPSL_FEM_POWER_REDUCE(6),
	.envelope.tx_power_ble[33] = MPSL_FEM_POWER_REDUCE(6),
	.envelope.tx_power_ble[34] = MPSL_FEM_POWER_REDUCE(6),
	.envelope.tx_power_ble[35] = MPSL_FEM_POWER_REDUCE(6),
	.envelope.tx_power_ble[36] = MPSL_FEM_POWER_REDUCE(12),
	.envelope.tx_power_ble[37] = MPSL_FEM_POWER_REDUCE(5),
	.envelope.tx_power_ble[38] = MPSL_FEM_POWER_REDUCE(5),
	.envelope.tx_power_ble[39] = MPSL_FEM_POWER_REDUCE(12),
};
#endif

#if defined(CONFIG_NRF_802154_SL)
const mpsl_tx_power_envelope_t ENV_802154 = {
	.phy = MPSL_PHY_Ieee802154_250Kbit,
	.envelope.tx_power_802154[0]  = MPSL_FEM_POWER_REDUCE(0), /* 11 */
	.envelope.tx_power_802154[1]  = MPSL_FEM_POWER_REDUCE(0),
	.envelope.tx_power_802154[2]  = MPSL_FEM_POWER_REDUCE(0),
	.envelope.tx_power_802154[3]  = MPSL_FEM_POWER_REDUCE(0),
	.envelope.tx_power_802154[4]  = MPSL_FEM_POWER_REDUCE(0),
	.envelope.tx_power_802154[5]  = MPSL_FEM_POWER_REDUCE(0),
	.envelope.tx_power_802154[6]  = MPSL_FEM_POWER_REDUCE(0),
	.envelope.tx_power_802154[7]  = MPSL_FEM_POWER_REDUCE(0), /* 18 */
	.envelope.tx_power_802154[8]  = MPSL_FEM_POWER_REDUCE(0),
	.envelope.tx_power_802154[9]  = MPSL_FEM_POWER_REDUCE(0),
	.envelope.tx_power_802154[10] = MPSL_FEM_POWER_REDUCE(0),
	.envelope.tx_power_802154[11] = MPSL_FEM_POWER_REDUCE(0),
	.envelope.tx_power_802154[12] = MPSL_FEM_POWER_REDUCE(0),
	.envelope.tx_power_802154[13] = MPSL_FEM_POWER_REDUCE(0),
	.envelope.tx_power_802154[14] = MPSL_FEM_POWER_REDUCE(8), /* 25 */
};
#endif

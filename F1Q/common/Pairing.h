// SPDX-License-Identifier: Apache-2.0
#pragma once
#include <stdint.h>
// Copy these definitions to Pairing.local.h and fill BOTH STA MAC addresses and
// two private random 16-byte keys. Use the same keys on both units. Never commit keys.
#if __has_include("Pairing.local.h")
#include "Pairing.local.h"
#else
static const uint8_t TIMER_MAC[6] = {};
static const uint8_t GROUND_MAC[6] = {};
static const uint8_t RADIO_PMK[16] = {};
static const uint8_t RADIO_LMK[16] = {};
#define F1Q_AP_PASSWORD "change-this-123"
#endif

/* -----------------------------------------------------------------------------
 * PPG Engine (MAX30102) – public API
 *  - Starts a 50 Hz sampler that reads IR/RED via Zephyr's MAX30102 driver
 *  - Maintains an internal ring buffer
 *  - On request, computes HR/SpO2 from the last 4 s window
 * -------------------------------------------------------------------------- */
#pragma once
#include <zephyr/device.h>
#include <stdbool.h>
#include <stdint.h>

/* Initialize and start sampling at ~50 Hz (returns 0 if running). */
int ppg_engine_init(void);

/* Stop sampling (optional). */
void ppg_engine_stop(void);

/* Get most recent estimates.
 * On success: returns 0, sets *hr_bpm / *spo2_pct and validity flags.
 * If invalid/noisy window: returns -EAGAIN.
 */
int ppg_engine_latest(uint8_t *hr_bpm, bool *hr_valid,
                      uint8_t *spo2_pct, bool *spo2_valid);

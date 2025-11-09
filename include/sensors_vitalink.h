/* -----------------------------------------------------------------------------
 * File: include/sensors_vitalink.h
 * Purpose: Thin sensor facade for VitalLink
 *  - Hides whether we use Zephyr drivers or raw I2C
 *  - Exposes init() and a single sensors_read() to the app
 *
 * Exposed data:
 *   - hr_bpm (MAX30102 or dummy)
 *   - spo2   (MAX30102 or dummy)
 *   - skin_c_x100 (MAX30208 via raw I2C, or dummy)
 *   - act_rms_x100 (LSM6DSOX or dummy)
 * --------------------------------------------------------------------------- */

#pragma once
#include <stdint.h>
#include <zephyr/device.h>

/* Initialize the sensor layer. Provide the I2C controller we’ll use for raw I2C. */
int sensors_init(const struct device *i2c_dev);

/* Read a single “snapshot” of vitals. Returns 0 on full success, <0 if any field is dummy. */
int sensors_read(uint8_t *hr_bpm,
                 uint8_t *spo2,
                 int16_t *skin_c_x100,
                 uint16_t *act_rms_x100);



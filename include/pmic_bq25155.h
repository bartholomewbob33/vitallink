/* -----------------------------------------------------------------------------
 * BQ25155 I2C helper (status + battery voltage)
 * --------------------------------------------------------------------------- */
#pragma once
#include <zephyr/device.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/* Typical TI addr if A0/A1 strapped low*/
#ifndef BQ25155_I2C_ADDR
#define BQ25155_I2C_ADDR 0x6B
#endif

/* ----registers/bits ---- */
#define BQ25155_REG_STATUS       0x0B  /* example */
#define  BQ25155_STATUS_USB_OK   (1u << 0)  /* example: VBUS/PG present */
#define  BQ25155_STATUS_CHG      (1u << 1)  /* example: charging active */
#define BQ25155_REG_BAT_MSB      0x14       /* example: VBAT MSB */
#define BQ25155_REG_BAT_LSB      0x15       /* example: VBAT LSB (upper 2 bits) */
/* Scaling example (adjust): 8 mV per LSB for a 10-bit value */
#define BQ25155_VBAT_LSB_MV      8

int bq25155_init(const struct device *i2c);
int bq25155_read_status(const struct device *i2c, bool *usb_present, bool *charging);
int bq25155_read_vbat_mv(const struct device *i2c, uint16_t *vbat_mv);

#ifdef __cplusplus
}
#endif

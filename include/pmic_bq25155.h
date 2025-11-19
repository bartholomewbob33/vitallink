/* -----------------------------------------------------------------------------
 * BQ25155 I2C helper (status + battery voltage + basic config)
 * --------------------------------------------------------------------------- */
#pragma once
#include <zephyr/device.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/* Typical TI addr if A0/A1 strapped low */
#ifndef BQ25155_I2C_ADDR
#define BQ25155_I2C_ADDR 0x6B
#endif

/* ---- registers / bits (from datasheet) ---- */
#define BQ25155_REG_STATUS       0x0B      /* Status register */
#define  BQ25155_STATUS_USB_OK   (1u << 0) /* VBUS/PG present */
#define  BQ25155_STATUS_CHG      (1u << 1) /* Charging active */

#define BQ25155_REG_BAT_MSB      0x14      /* VBAT MSB */
#define BQ25155_REG_BAT_LSB      0x15      /* VBAT LSB (upper 2 bits) */
/* Scaling: 8 mV per LSB for a 10-bit value */
#define BQ25155_VBAT_LSB_MV      8

/* New: ILIM and LDO control registers */
#define BQ25155_REG_ILIMCTRL     0x19      /* Input current limit control */
#define BQ25155_REG_LDOCTRL      0x1D      /* LDO / LS control */

/* ILIMCTRL ILIM_2:0 codes (Table 9-28) */
#define BQ25155_ILIM_50MA        0b000
#define BQ25155_ILIM_100MA       0b001  /* used 100 ma instead of 75 ma since there was no 75 (closest to requested 75 mA) */
#define BQ25155_ILIM_150MA       0b010
#define BQ25155_ILIM_200MA       0b011
#define BQ25155_ILIM_300MA       0b100
#define BQ25155_ILIM_400MA       0b101
#define BQ25155_ILIM_500MA       0b110
#define BQ25155_ILIM_600MA       0b111

/* LDOCTRL VLDO_4:0 code for 3.3 V
 * From datasheet: V_LDO = 600 mV + VLDO_code * 100 mV
 * 3.3 V → (3.3 - 0.6) / 0.1 = 27 → 0b11011
 */
#define BQ25155_LDO_V_3V3_CODE   27u

int bq25155_init(const struct device *i2c);
int bq25155_read_status(const struct device *i2c, bool *usb_present, bool *charging);
int bq25155_read_vbat_mv(const struct device *i2c, uint16_t *vbat_mv);

#ifdef __cplusplus
}
#endif

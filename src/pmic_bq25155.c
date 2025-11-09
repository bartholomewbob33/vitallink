/* -----------------------------------------------------------------------------
 * Minimal BQ25155 I2C helper (status + battery voltage)
 * Replace register addresses/scale with values from your BQ25155 datasheet.
 * --------------------------------------------------------------------------- */
#include <zephyr/drivers/i2c.h>
#include <zephyr/logging/log.h>
#include "pmic_bq25155.h"

LOG_MODULE_REGISTER(bq25155, LOG_LEVEL_INF);

static inline int r8(const struct device *i2c, uint8_t reg, uint8_t *val)
{
	return i2c_reg_read_byte(i2c, BQ25155_I2C_ADDR, reg, val);
}

int bq25155_init(const struct device *i2c)
{
	if (!i2c || !device_is_ready(i2c)) return -ENODEV;
	/* If you need to configure LDO voltage/current limits, do writes here. */
	return 0;
}

int bq25155_read_status(const struct device *i2c, bool *usb_present, bool *charging)
{
	uint8_t s = 0;
	int err = r8(i2c, BQ25155_REG_STATUS, &s);
	if (err) return err;

	if (usb_present) *usb_present = (s & BQ25155_STATUS_USB_OK) != 0;
	if (charging)    *charging    = (s & BQ25155_STATUS_CHG)    != 0;
	return 0;
}

int bq25155_read_vbat_mv(const struct device *i2c, uint16_t *vbat_mv)
{
	/* Example: 10-bit VBAT in MSB[7:0] + LSB[7:6], 8 mV/LSB */
	uint8_t msb=0, lsb=0;
	int err = r8(i2c, BQ25155_REG_BAT_MSB, &msb);
	if (err) return err;
	err = r8(i2c, BQ25155_REG_BAT_LSB, &lsb);
	if (err) return err;

	uint16_t raw = ((uint16_t)msb << 2) | ((lsb >> 6) & 0x03);
	uint32_t mv = (uint32_t)raw * (uint32_t)BQ25155_VBAT_LSB_MV;

	if (vbat_mv) *vbat_mv = (uint16_t)mv;
	return 0;
}

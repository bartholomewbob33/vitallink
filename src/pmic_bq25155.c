/* -----------------------------------------------------------------------------
 * Minimal BQ25155 I2C helper (status + battery voltage + basic config)
 * --------------------------------------------------------------------------- */
#include <zephyr/drivers/i2c.h>
#include <zephyr/logging/log.h>
#include "pmic_bq25155.h"

LOG_MODULE_REGISTER(bq25155, LOG_LEVEL_INF);

static inline int r8(const struct device *i2c, uint8_t reg, uint8_t *val)
{
    return i2c_reg_read_byte(i2c, BQ25155_I2C_ADDR, reg, val);
}

static inline int w8(const struct device *i2c, uint8_t reg, uint8_t val)
{
    return i2c_reg_write_byte(i2c, BQ25155_I2C_ADDR, reg, val);
}

int bq25155_init(const struct device *i2c)
{
    if (!i2c || !device_is_ready(i2c)) {
        return -ENODEV;
    }

    /* ----------------------------------------------------------
     * 1) Configure input current limit (ILIM)
     *    Spec asked for ~75 mA; the chip only supports discrete
     *    steps, so we choose 100 mA (ILIM_2:0 = 0b001).
     * ---------------------------------------------------------- */
    uint8_t ilim_val = BQ25155_ILIM_100MA;
    int err = w8(i2c, BQ25155_REG_ILIMCTRL, ilim_val);
    if (err) {
        LOG_ERR("BQ25155: ILIM write failed (%d)", err);
        return err;
    }

    /* ----------------------------------------------------------
     * 2) Configure LDO output to 3.3 V
     *
     *    Datasheet: V_LDO = 600 mV + VLDO_code * 100 mV
     *    For 3.3 V → VLDO_code = (3.3 - 0.6) / 0.1 = 27 (0b11011)
     *
     *    LDOCTRL (0x1D):
     *      bit 7   EN_LS_LDO        = 1 (enable LDO)
     *      bits 6:2 VLDO_4:0        = code (27)
     *      bit 1   LDO_SWITCH_CONF  = 0 (LDO mode, not load switch)
     *      bit 0   RESERVED         = 0
     * ---------------------------------------------------------- */
    uint8_t ldo_code = (uint8_t)(BQ25155_LDO_V_3V3_CODE & 0x1F);
    uint8_t ldo_val =
        (1u << 7) |          /* EN_LS_LDO = 1 (enable) */
        (ldo_code << 2) |    /* VLDO_4:0 code for 3.3 V */
        (0u << 1);           /* LDO mode, not load switch */

    err = w8(i2c, BQ25155_REG_LDOCTRL, ldo_val);
    if (err) {
        LOG_ERR("BQ25155: LDO write failed (%d)", err);
        return err;
    }

    LOG_INF("BQ25155 init: ILIM=100mA, LDO=3.3V (code=%u)", ldo_code);
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
    /* 10-bit VBAT in MSB[7:0] + LSB[7:6], 8 mV/LSB */
    uint8_t msb = 0, lsb = 0;
    int err = r8(i2c, BQ25155_REG_BAT_MSB, &msb);
    if (err) return err;
    err = r8(i2c, BQ25155_REG_BAT_LSB, &lsb);
    if (err) return err;

    uint16_t raw = ((uint16_t)msb << 2) | ((lsb >> 6) & 0x03);
    uint32_t mv  = (uint32_t)raw * (uint32_t)BQ25155_VBAT_LSB_MV;

    if (vbat_mv) *vbat_mv = (uint16_t)mv;
    return 0;
}

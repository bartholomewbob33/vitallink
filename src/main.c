/* -----------------------------------------------------------------------------
 * File: src/main.c
 * Purpose: Minimal wristband firmware skeleton for nRF5340 DK (NCS v2.9.2).
 * Features:
 *  - Initializes Bluetooth in Peripheral role
 *  - Advertises as "VitalLink-Band"
 *  - Exposes a custom service + NOTIFY characteristic
 *  - Sends a 16-byte "vitals" packet once per second (dummy values for now)
 *  - I2C is enabled; real sensor reads will be added later
 * Usage:
 *  - Build for board target: nrf5340dk/nrf5340/cpuapp
 *  - Flash to DK; use a phone app (nRF Connect) or Pi to subscribe and view data
 * TODO (later):
 *  - Replace dummy values with real I2C reads (MAX30102, MAX30208, LSM6DSOX)
 * --------------------------------------------------------------------------- */

#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <zephyr/drivers/i2c.h>
#include <zephyr/bluetooth/bluetooth.h>
#include <zephyr/bluetooth/gatt.h>
#include <zephyr/logging/log.h>

#include "vitalink_uuids.h"
#include "vitalink_proto.h"

LOG_MODULE_REGISTER(vitalink, LOG_LEVEL_INF);

/* ----- I2C handle (for sensors later) ------------------------------------ */
#define I2C_NODE DT_ALIAS(i2c_ppg)
static const struct device *i2c0_dev;  

/* ----- BLE UUIDs (service + characteristic) ------------------------------- */
static struct bt_uuid_128 svc_uuid = VITALINK_SERVICE_UUID_INIT;
static struct bt_uuid_128 chr_uuid = VITALS_CHAR_UUID_INIT;

/* Client Characteristic Configuration (CCC) flag: true when a central subscribed */
static uint8_t notify_enabled;

/* CCC change callback: runs when a central enables/disables notifications */
static void ccc_cfg_changed(const struct bt_gatt_attr *attr, uint16_t value)
{
  notify_enabled = (value == BT_GATT_CCC_NOTIFY);
  LOG_INF("Notify %s", notify_enabled ? "ENABLED" : "DISABLED");
}

/* Our primary service: one characteristic that supports NOTIFY */
BT_GATT_SERVICE_DEFINE(vitalink_svc,
  BT_GATT_PRIMARY_SERVICE(&svc_uuid),
  BT_GATT_CHARACTERISTIC(&chr_uuid.uuid,
                         BT_GATT_CHRC_NOTIFY,      /* properties */
                         BT_GATT_PERM_NONE,        /* security on value (none for now) */
                         NULL, NULL, NULL),        /* read, write, user_data */
  BT_GATT_CCC(ccc_cfg_changed, BT_GATT_PERM_READ | BT_GATT_PERM_WRITE)
);

/* BLE ready callback: start advertising so centrals (Pi/phone) can find us */
static void bt_ready_cb(int err)
{
  if (err) {
    LOG_ERR("Bluetooth init failed (%d)", err);
    return;
  }

  const struct bt_data ad[] = {
    BT_DATA_BYTES(BT_DATA_FLAGS, (BT_LE_AD_GENERAL | BT_LE_AD_NO_BREDR)),
    BT_DATA(BT_DATA_NAME_COMPLETE, CONFIG_BT_DEVICE_NAME,
            sizeof(CONFIG_BT_DEVICE_NAME) - 1),
  };

  err = bt_le_adv_start(BT_LE_ADV_CONN, ad, ARRAY_SIZE(ad), NULL, 0);
  if (err) {
    LOG_ERR("Advertising start failed (%d)", err);
  } else {
    LOG_INF("Advertising as \"%s\"", CONFIG_BT_DEVICE_NAME);
  }
}

/* Simple CRC-8 (poly 0x07) over bytes [0..13] for integrity */
static uint8_t crc8(const uint8_t *p, size_t n)
{
  uint8_t c = 0;
  for (size_t i = 0; i < n; i++) {
    c ^= p[i];
    for (int b = 0; b < 8; b++) {
      c = (c & 0x80) ? ((c << 1) ^ 0x07) : (c << 1);
    }
  }
  return c;
}

/* Work item: runs once per second; builds a packet and notifies if subscribed */
static struct k_work_delayable tick_work;
static uint16_t seq;

static void send_vitals(const struct vitals_pkt *pkt)
{
  if (!notify_enabled) { return; }

  /* Our characteristic value attribute is index 1 in the service table above */
  const struct bt_gatt_attr *attr = &vitalink_svc.attrs[1];
  int err = bt_gatt_notify(NULL, attr, pkt, sizeof(*pkt));
  if (err) {
    LOG_WRN("notify err %d", err);
  }
}

static void tick_fn(struct k_work *work)
{
  ARG_UNUSED(work);

  struct vitals_pkt p = {0};

  p.ver   = 1;                       /* protocol version */
  p.seq   = seq++;                   /* increments every packet */
  p.ts_ms = k_uptime_get_32();       /* ms since boot */
  p.flags = 0;                       /* no alerts in dummy data */

  /* ----- DUMMY VALUES for bring-up ---------------------------------------
   * TODO: replace these with real sensor reads via I2C:
   *   - MAX30102: compute HR/SpO2 or send raw averages initially
   *   - MAX30208: read temperature register, convert to x100C
   *   - LSM6DSOX: compute a simple activity metric (RMS magnitude)
   * --------------------------------------------------------------------- */
  p.hr_bpm       = 76;               /* e.g., 76 BPM */
  p.spo2         = 98;               /* e.g., 98% */
  p.skin_c_x100  = 3650;             /* 36.50 C */
  p.act_rms_x100 = 180;              /* 1.80 (scaled) */

  /* CRC across first 14 bytes (exclude crc8 and rfu fields) */
  p.crc8 = crc8((const uint8_t *)&p, 14);

  /* Push it to the central if subscribed */
  send_vitals(&p);

  /* Reschedule for 1 second later (notify rate = 1 Hz) */
  k_work_schedule(&tick_work, K_SECONDS(1));
}

void main(void)
{
  /* I2C device handle (for later sensor work) */
  i2c0_dev = DEVICE_DT_GET(I2C_NODE);
  if (!device_is_ready(i2c0_dev)) {
    LOG_WRN("I2C0 not ready (OK for now; sensors not used yet)");
  }

  /* Start Bluetooth stack; bt_ready_cb will start advertising on success */
  int err = bt_enable(bt_ready_cb);
  if (err) {
    LOG_ERR("bt_enable failed: %d", err);
    return;
  }

  /* Kick off our 1 Hz publisher */
  k_work_init_delayable(&tick_work, tick_fn);
  k_work_schedule(&tick_work, K_SECONDS(1));
}

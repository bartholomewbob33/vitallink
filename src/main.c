/* -----------------------------------------------------------------------------
 * File: src/main.c
 * Purpose: VitalLink wristband firmware (nRF5340 DK, NCS v2.9.2)
 *
 * What this file does:
 *  - Initializes Bluetooth in Peripheral role.
 *  - Sets a DYNAMIC device name at runtime using the BLE address tail
 *    (e.g., "VitalLink-3F9A").
 *  - Defines a custom GATT service + NOTIFY characteristic (UUIDs in
 *    vitalink_uuids.h).
 *  - Builds a 16-byte vitals packet once per second and decides a tier:
 *        NORMAL   => do NOT send every second, only send a heartbeat every 60 s
 *        ALERT    => send immediately
 *        EMERGENCY=> send immediately and beep the active buzzer (if present)
 *  - Uses an 8-bit CHECKSUM (sum of bytes 0..13) to protect the packet.
 *
 * New minimal-but-important improvements in this version:
 *  - Fix: notify uses the correct GATT attribute (Characteristic Value).
 *  - Advertising now includes the 128-bit Service UUID (scanner-friendly).
 *  - Buzzer pulse no longer sleeps the system workqueue (uses delayed work).
 *  - Uses protocol helpers: VITALINK_PROTO_VER + vitals_finalize().
 *  - Better bt_id_get() error handling; clarified I²C device naming.
 *
 * Not implemented yet:
 *  - Real sensor reads (MAX30102, MAX30208, LSM6DSOX via I2C).
 *  - GPIO interrupt wiring for sensors (we are sampling at 1 Hz regardless).
 *
 * Optional buzzer:
 *  - If board overlay defines an alias "buzzer" (see overlay template), we
 *    configure that GPIO and toggle it on EMERGENCY tiers without blocking.
 *
 * Build target:
 *  - Board: nrf5340dk/nrf5340/cpuapp
 * --------------------------------------------------------------------------- */

#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <zephyr/drivers/i2c.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/bluetooth/bluetooth.h>
#include <zephyr/bluetooth/gatt.h>
#include <zephyr/logging/log.h>
#include <stdbool.h>                 /* bool */

#include "vitalink_uuids.h"          /* UUID objects, accessors, ADV helper */
#include "vitalink_proto.h"          /* struct vitals_pkt and helpers */

LOG_MODULE_REGISTER(vitalink, LOG_LEVEL_INF);

/* ===== I2C handle (for sensors later) ==================================== */
/* DT alias must exist in your overlay:
 *   &{/aliases} { i2c-ppg = &i2c1; };
 */
#define I2C_NODE DT_ALIAS(i2c_ppg)
static const struct device *i2c_dev;

/* ===== Optional buzzer (active buzzer via GPIO) =========================== */
/* If your DTS overlay defines an alias "buzzer", we use it. Otherwise we compile
 * without buzzer support and nothing breaks.
 */
#if DT_NODE_HAS_STATUS(DT_ALIAS(buzzer), okay)
#define HAS_BUZZER 1
static const struct gpio_dt_spec buzzer = GPIO_DT_SPEC_GET(DT_ALIAS(buzzer), gpios);
#else
#define HAS_BUZZER 0
#endif

/* ===== BLE GATT: Service + Characteristic with NOTIFY ===================== */
/* Use the exported UUID accessors so there is a single source of truth.      */
static uint8_t notify_enabled;  /* CCC state: 1 when central subscribed */

/* CCC change callback: runs when a central enables/disables notifications */
static void ccc_cfg_changed(const struct bt_gatt_attr *attr, uint16_t value)
{
    notify_enabled = (value == BT_GATT_CCC_NOTIFY);
    LOG_INF("Notify %s", notify_enabled ? "ENABLED" : "DISABLED");
}

/* Our primary service: one characteristic that supports NOTIFY
 * Attribute indices inside this service instance:
 *   [0] Primary Service declaration
 *   [1] Characteristic declaration
 *   [2] Characteristic VALUE  <-- notify must target THIS attribute
 *   [3] CCC descriptor
 */

static const struct bt_uuid_128 SVC_UUID = VITALINK_SERVICE_UUID_INIT;
static const struct bt_uuid_128 CHR_UUID = VITALS_CHAR_UUID_INIT;

BT_GATT_SERVICE_DEFINE(vitalink_svc,
    BT_GATT_PRIMARY_SERVICE(&SVC_UUID.uuid),
    BT_GATT_CHARACTERISTIC(&CHR_UUID.uuid,
                           BT_GATT_CHRC_NOTIFY,
                           BT_GATT_PERM_NONE,
                           NULL, NULL, NULL),
    BT_GATT_CCC(ccc_cfg_changed, BT_GATT_PERM_READ | BT_GATT_PERM_WRITE)
);

/* Convenience macro to make it obvious which attribute to notify on. */
#define VITALINK_CHAR_VALUE_ATTR (&vitalink_svc.attrs[2])

/* ===== BLE dynamic name helpers ===========================================
 * How the dynamic name works:
 *  - call bt_id_get() to read the controller's identity address
 *  - take the last two bytes and format "VitalLink-%02X%02X"
 *  - call bt_set_name() before advertising to apply it
 */

#define VITALINK_DYN_NAME_MAX 32  /* local buffer: avoids depending on Kconfig sizes */

/* Build a dynamic name like "VitalLink-3F9A" using last 2 bytes of the BLE address.
 * Must be called BEFORE advertising starts.
 */
static void set_dynamic_name_from_addr(void)
{
    bt_addr_le_t addrs[CONFIG_BT_ID_MAX];
    size_t count = CONFIG_BT_ID_MAX;

    /* In NCS 2.9.x this is 'void'; it fills addrs[] and updates count */
    bt_id_get(addrs, &count);

    if (count == 0) {
        LOG_WRN("No BLE identities; keeping default name");
        return;
    }

    const uint8_t *v = addrs[0].a.val; /* 6 bytes */
    char dyn_name[VITALINK_DYN_NAME_MAX];

    snprintk(dyn_name, sizeof(dyn_name), "VitalLink-%02X%02X", v[1], v[0]);

    int err = bt_set_name(dyn_name);
    if (err) {
        LOG_WRN("bt_set_name failed (%d)", err);
    } else {
        LOG_INF("BLE name set to: %s", dyn_name);
    }
}

/* BLE ready callback: start advertising so centrals (Pi/phone) can find us.
 * NEW: include the 128-bit Service UUID in advertising so the Pi can filter.
 */
static void bt_ready_cb(int err)
{
    if (err) {
        LOG_ERR("Bluetooth init failed (%d)", err);
        return;
    }

    /* Advertising payload:
     *  - Flags (General Disc, no BR/EDR)
     *  - Our 128-bit Service UUID (so scanners can filter quickly)
     * The device name is auto-included by BT_LE_ADV_CONN_NAME.
     */
    static const struct bt_data ad[] = {
        BT_DATA_BYTES(BT_DATA_FLAGS, (BT_LE_AD_GENERAL | BT_LE_AD_NO_BREDR)),
        VITALINK_AD_SERVICE_UUID,
    };

    err = bt_le_adv_start(BT_LE_ADV_CONN_NAME, ad, ARRAY_SIZE(ad), NULL, 0);
    if (err) {
        LOG_ERR("Advertising start failed (%d)", err);
    } else {
        LOG_INF("Advertising started");
    }
}

/* ===== Tiering thresholds and policy ====================================== */
/* Thresholds: adjust later to clinically meaningful values. */
#define HR_ALERT_BPM        120
#define HR_EMERG_BPM        140
#define SPO2_ALERT_PCT       92
#define SPO2_EMERG_PCT       88
#define TEMP_ALERT_C_X100  3800   /* 38.00 C */
#define TEMP_EMERG_C_X100  3950   /* 39.50 C */

/* Optional buzzer timing (ms) */
#define BUZZER_ON_MS_EMERG  300

/* Decide flags for current sample (EMERGENCY dominates, then ALERT). */
static uint8_t decide_flags(uint8_t hr_bpm, uint8_t spo2, int16_t skin_c_x100)
{
    uint8_t flags = 0;

    /* Any EMERGENCY condition dominates */
    if (hr_bpm >= HR_EMERG_BPM || spo2 <= SPO2_EMERG_PCT || skin_c_x100 >= TEMP_EMERG_C_X100) {
        flags |= VFLAG_EMERGENCY;
        return flags;
    }

    /* Otherwise, any ALERT condition */
    if (hr_bpm >= HR_ALERT_BPM || spo2 <= SPO2_ALERT_PCT || skin_c_x100 >= TEMP_ALERT_C_X100) {
        flags |= VFLAG_ALERT;
        return flags;
    }

    /* NORMAL => no alert flags set */
    return flags;
}

/* ===== Non-blocking buzzer pulse ========================================== */
/* NEW: Use a delayed work item to turn the buzzer off later, avoiding k_sleep
 * in the system workqueue (which would block other work items).
 */
#if HAS_BUZZER
static struct k_work_delayable buzzer_off_work;

static void buzzer_off_fn(struct k_work *w)
{
    ARG_UNUSED(w);
    gpio_pin_set_dt(&buzzer, 0);  /* ensure off */
}

/* Trigger a short pulse for EMERGENCY tier without blocking. */
static void buzzer_pulse_emergency(void)
{
    if (!device_is_ready(buzzer.port)) {
        return;
    }
    gpio_pin_set_dt(&buzzer, 1);
    k_work_reschedule(&buzzer_off_work, K_MSEC(BUZZER_ON_MS_EMERG));
}
#else
static void buzzer_pulse_emergency(void) { /* no-op when buzzer is absent */ }
#endif

/* ===== Notify helper ======================================================= */
/* IMPORTANT: notify must target the Characteristic VALUE attribute. */
static void send_vitals(const struct vitals_pkt *pkt)
{
    if (!notify_enabled) { return; }

    const struct bt_gatt_attr *attr = VITALINK_CHAR_VALUE_ATTR; /* value at index 2 */
    int err = bt_gatt_notify(NULL, attr, pkt, sizeof(*pkt));
    if (err) {
        LOG_WRN("notify err %d", err);
    }
}

/* ===== 1 Hz work item ===================================================== */
static struct k_work_delayable tick_work;
static uint16_t seq;

/* suppress NORMAL packets most of the time but send a signal every 60 s. */
#define NORMAL_HEARTBEAT_MS  (60 * 1000)
static uint32_t last_normal_sent_ms = 0;

static void tick_fn(struct k_work *work)
{
    ARG_UNUSED(work);

    /* -----------------------------------------
     * TODO: Replace dummy values with real reads
     * -----------------------------------------
     * MAX30102: Read FIFO, compute HR/SpO2 (or averaged raw to start).
     * MAX30208: Read temperature register, convert to centi-degC.
     * LSM6DSOX: Read accel, compute RMS magnitude for simple activity.
     */
    uint8_t  hr_bpm       = 76;    /* dummy */
    uint8_t  spo2         = 98;    /* dummy */
    int16_t  skin_c_x100  = 3650;  /* dummy 36.50 C */
    uint16_t act_rms_x100 = 180;   /* dummy 1.80 */

    /* Decide alert flags for this sample */
    uint8_t flags = decide_flags(hr_bpm, spo2, skin_c_x100);

    /* Build packet */
    struct vitals_pkt p = {0};
    p.seq   = seq++;                   /* increments every packet */
    p.ts_ms = k_uptime_get_32();       /* ms since boot */
    p.flags = flags;

    p.hr_bpm       = hr_bpm;
    p.spo2         = spo2;
    p.skin_c_x100  = skin_c_x100;
    p.act_rms_x100 = act_rms_x100;

    /* NEW: finalize sets version + zeros RFU + computes checksum */
    vitals_finalize(&p);

    /* Decide whether to send based on policy */
    bool should_send = false;

    if ((flags & (VFLAG_ALERT | VFLAG_EMERGENCY)) != 0) {
        /* ALERT or EMERGENCY => always send */
        should_send = true;

        if (flags & VFLAG_EMERGENCY) {
            buzzer_pulse_emergency();
        }
    } else {
        /* NORMAL: send only if heartbeat interval elapsed */
        uint32_t now = p.ts_ms;
        if ((now - last_normal_sent_ms) >= NORMAL_HEARTBEAT_MS) {
            should_send = true;
            last_normal_sent_ms = now;
        }
    }

    if (should_send) {
        send_vitals(&p);
    }

    /* Reschedule for 1 second later (sample rate = 1 Hz) */
    k_work_reschedule(&tick_work, K_SECONDS(1));
}

/* ===== App entry =========================================================== */
void main(void)
{
    /* I2C device handle (for later sensor work) */
    i2c_dev = DEVICE_DT_GET(I2C_NODE);
    if (!device_is_ready(i2c_dev)) {
        LOG_WRN("I2C device not ready (OK for now; sensors not used yet)");
    }

#if HAS_BUZZER
    /* Configure buzzer GPIO if present */
    if (!device_is_ready(buzzer.port)) {
        LOG_WRN("Buzzer GPIO not ready");
    } else {
        int err = gpio_pin_configure_dt(&buzzer, GPIO_OUTPUT_INACTIVE);
        if (err) {
            LOG_WRN("Buzzer gpio config failed (%d)", err);
        }
        /* Init buzzer off-work so pulses don't block workqueue */
        k_work_init_delayable(&buzzer_off_work, buzzer_off_fn);
    }
#endif

    /* Set dynamic BLE device name BEFORE advertising */
    set_dynamic_name_from_addr();

    /* Start Bluetooth stack; bt_ready_cb will start advertising on success */
    int err = bt_enable(bt_ready_cb);
    if (err) {
        LOG_ERR("bt_enable failed: %d", err);
        return;
    }

    /* start 1 Hz sampler/publisher */
    k_work_init_delayable(&tick_work, tick_fn);
    k_work_schedule(&tick_work, K_SECONDS(1));
}

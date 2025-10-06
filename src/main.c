/* -----------------------------------------------------------------------------
 * File: src/main.c
 * Purpose: VitalLink wristband firmware (nRF5340 DK, NCS v2.9.2)
 *
 * What this file does:
 *  - Initializes Bluetooth in Peripheral role.
 *  - Sets a DYNAMIC device name at runtime using the BLE address tail (e.g., "VitalLink-3F9A").
 *  - Defines a custom GATT service + NOTIFY characteristic (UUIDs in vitalink_uuids.h).
 *  - Builds a 16-byte vitals packet once per second and decides a tier:
 *        NORMAL   => do NOT send every second, only send a heartbeat every 60 s
 *        ALERT    => send immediately
 *        EMERGENCY=> send immediately and beep the active buzzer (if present)
 *  - Uses an 8-bit CHECKSUM (sum of bytes 0..13) to protect the packet.
 *
 * Not implemented yet:
 *  - Real sensor reads (MAX30102, MAX30208, LSM6DSOX via I2C).
 *  - GPIO interrupt wiring for sensors (we are sampling at 1 Hz regardless).
 *
 * Optional buzzer:
 *  - If board overlay defines an alias "buzzer", e.g.:
 *      &gpio0 { buzzer0: buzzer0 { gpios = <&gpio0 15 GPIO_ACTIVE_HIGH>; status="okay"; }; };
 *      / { aliases { buzzer = &buzzer0; }; };
 *    then this code will configure that GPIO and toggle it on EMERGENCY tiers.
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
#include <stdbool.h>                 /* <-- required for 'bool' */

#include "vitalink_uuids.h"
#include "vitalink_proto.h"          /* struct vitals_pkt and VFLAG_* bits */

LOG_MODULE_REGISTER(vitalink, LOG_LEVEL_INF);

/* ===== I2C handle (for sensors later) ==================================== */
/* DT alias must exist in your overlay:
 *   &{/aliases} { i2c-ppg = &i2c1; };
 */
#define I2C_NODE DT_ALIAS(i2c_ppg)
static const struct device *i2c0_dev;

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

/* ===== BLE UUIDs (service + characteristic) =============================== */
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

/* ===== BLE dynamic name helpers ===========================================
 * How the dynamic name works:
 *  - call bt_id_get() to read the controller's identity address
 *  - take the last two bytes and format "VitalLink-%02X%02X"
 *  - call bt_set_name() before advertising to apply it
 */

/* We use a small local buffer to avoid relying on editor-unfriendly Kconfig macros. */
#define VITALINK_DYN_NAME_MAX 32

/* Build a dynamic name like "VitalLink-3F9A" using last 2 bytes of the BLE address.
 * Must be called BEFORE advertising starts.
 */
static void set_dynamic_name_from_addr(void)
{
    bt_addr_le_t addrs[CONFIG_BT_ID_MAX];
    size_t count = CONFIG_BT_ID_MAX;

    /* bt_id_get() is void; it fills the array and count. */
    bt_id_get(addrs, &count);

    if (count == 0) {
        LOG_WRN("No BLE identities found; keeping default name");
        return;
    }

    /* Use the first identity address (index 0). Take the last 2 bytes for readability. */
    const uint8_t *v = addrs[0].a.val; /* 6 bytes, little-endian order */
    char dyn_name[VITALINK_DYN_NAME_MAX];

    /* Example: VitalLink-3F9A (using bytes [1] and [0]) */
    snprintk(dyn_name, sizeof(dyn_name), "VitalLink-%02X%02X", v[1], v[0]);

    int err = bt_set_name(dyn_name);
    if (err) {
        LOG_WRN("bt_set_name failed (%d)", err);
    } else {
        LOG_INF("BLE name set to: %s", dyn_name);
    }
}

/* BLE ready callback: start advertising so centrals (Pi/phone) can find us */
static void bt_ready_cb(int err)
{
    if (err) {
        LOG_ERR("Bluetooth init failed (%d)", err);
        return;
    }

    /* Include the (possibly dynamic) device name in advertising automatically. */
    err = bt_le_adv_start(BT_LE_ADV_CONN_NAME, NULL, 0, NULL, 0);
    if (err) {
        LOG_ERR("Advertising start failed (%d)", err);
    } else {
        LOG_INF("Advertising started");
    }
}

/* ===== Packet integrity: 8-bit checksum =================================== */
/* Sum bytes [0..n-1] and return (sum & 0xFF). This must match the Pi parser. */
static uint8_t checksum8(const uint8_t *p, size_t n)
{
    uint32_t sum = 0;
    for (size_t i = 0; i < n; i++) {
        sum += p[i];
    }
    return (uint8_t)(sum & 0xFF);
}

/* ===== Tiering thresholds and policy ====================================== */
/* We now use the flags bits (VFLAG_ALERT, VFLAG_EMERGENCY) from vitalink_proto.h.
 * NORMAL: neither flag is set.
 * ALERT:  VFLAG_ALERT set.
 * EMERG:  VFLAG_EMERGENCY set (and we also beep if buzzer is present).
 *
 * Thresholds: adjust later to clinically meaningful values.
 */
#define HR_ALERT_BPM        120
#define HR_EMERG_BPM        140
#define SPO2_ALERT_PCT       92
#define SPO2_EMERG_PCT       88
#define TEMP_ALERT_C_X100  3800   /* 38.00 C */
#define TEMP_EMERG_C_X100  3950   /* 39.50 C */

/* Optional buzzer timings (ms) */
#define BUZZER_ON_MS_EMERG  300

/* Decide flags for current sample */
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

static void buzzer_pulse_emergency(void)
{
#if HAS_BUZZER
    if (!device_is_ready(buzzer.port)) {
        return;
    }
    /* One short pulse; keep it simple for now */
    gpio_pin_set_dt(&buzzer, 1);
    k_sleep(K_MSEC(BUZZER_ON_MS_EMERG));
    gpio_pin_set_dt(&buzzer, 0);
#endif
}

/* ===== Notify helper ======================================================= */
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

/* ===== 1 Hz work item ===================================================== */
static struct k_work_delayable tick_work;
static uint16_t seq;

/*  suppress NORMAL packets most of the time but send a signal every 60 s. */
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
    p.ver   = 1;                       /* protocol version */
    p.seq   = seq++;                   /* increments every packet */
    p.ts_ms = k_uptime_get_32();       /* ms since boot */
    p.flags = flags;

    p.hr_bpm       = hr_bpm;
    p.spo2         = spo2;
    p.skin_c_x100  = skin_c_x100;
    p.act_rms_x100 = act_rms_x100;

    /* Compute checksum across first 14 bytes (exclude checksum and rfu) */
    p.checksum = checksum8((const uint8_t *)&p, 14);

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
    k_work_schedule(&tick_work, K_SECONDS(1));
}

/* ===== App entry =========================================================== */
void main(void)
{
    /* I2C device handle (for later sensor work) */
    i2c0_dev = DEVICE_DT_GET(I2C_NODE);
    if (!device_is_ready(i2c0_dev)) {
        LOG_WRN("I2C0 not ready (OK for now; sensors not used yet)");
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
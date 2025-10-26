/* -----------------------------------------------------------------------------
 * File: src/main.c
 * Purpose: VitalLink wristband firmware (nRF5340 DK, NCS v2.9.2)
 *
 * What this file does:
 *  - Initializes Bluetooth in Peripheral role.
 *  - Sets a DYNAMIC device name at runtime using the BLE address tail.
 *  - Defines a custom GATT service + NOTIFY characteristic.
 *  - Builds a 16-byte vitals packet once per second and decides a tier:
 *        NORMAL   => suppress except 60 s heartbeats
 *        ALERT    => send immediately
 *        EMERGENCY=> send immediately + buzzer pulse (if present)
 *  - Uses an 8-bit CHECKSUM (sum of bytes 0..13).
 *  -  additional flags:
 *        Motion artifact (debounced)
 *        Low battery (via ADC if VBAT alias present)
 *        Sensor fault (debounced failure counters)
 *  - Implements "Charging ⇒ System OFF":
 *        If the charger STAT (active-low) is asserted at boot, enter System OFF.
 *        If STAT asserts during runtime, quickly enter System OFF as well.
 *
 * Build target:
 *  - Board: nrf5340dk/nrf5340/cpuapp
 * --------------------------------------------------------------------------- */

#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <zephyr/drivers/i2c.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/adc.h>
#include <zephyr/bluetooth/bluetooth.h>
#include <zephyr/bluetooth/gatt.h>
#include <zephyr/logging/log.h>
#include <stdbool.h>

#include "vitalink_uuids.h"
#include "vitalink_proto.h"

LOG_MODULE_REGISTER(vitalink, LOG_LEVEL_INF);

/* ===== I2C handle (for sensors later) ==================================== */
#define I2C_NODE DT_ALIAS(i2c_ppg)
static const struct device *i2c_dev;

/* ===== Optional buzzer (active buzzer via GPIO) =========================== */
#if DT_NODE_HAS_STATUS(DT_ALIAS(buzzer), okay)
#define HAS_BUZZER 1
static const struct gpio_dt_spec buzzer = GPIO_DT_SPEC_GET(DT_ALIAS(buzzer), gpios);
#else
#define HAS_BUZZER 0
#endif

/* ===== Charging detect (PMIC STAT, active-low) ============================ */
#if DT_NODE_HAS_STATUS(DT_ALIAS(chg_stat), okay)
#define HAS_CHG_STAT 1
static const struct gpio_dt_spec chg_stat = GPIO_DT_SPEC_GET(DT_ALIAS(chg_stat), gpios);
#else
#define HAS_CHG_STAT 0
#endif

/* =====  VBAT ADC alias (external divider) =========================
 * If this alias is present, we can compute true VBAT and set the low-battery flag.
 * If absent, low-battery logic is a clean no-op until wired.
 */
#if DT_NODE_HAS_STATUS(DT_ALIAS(vbat), okay)
#define HAS_VBAT_ADC 1
/* The alias points to a node with io-channels = <&adc N>; we fetch that channel. */
static const struct adc_dt_spec vbat_adc = ADC_DT_SPEC_GET(DT_ALIAS(vbat));
#else
#define HAS_VBAT_ADC 0
#endif

/* ===== BLE GATT: Service + Characteristic (static UUID objects) ========== */
static const struct bt_uuid_128 SVC_UUID = VITALINK_SERVICE_UUID_INIT;
static const struct bt_uuid_128 CHR_UUID = VITALS_CHAR_UUID_INIT;
static uint8_t notify_enabled;

/* CCC change callback */
static void ccc_cfg_changed(const struct bt_gatt_attr *attr, uint16_t value)
{
    notify_enabled = (value == BT_GATT_CCC_NOTIFY);
    LOG_INF("Notify %s", notify_enabled ? "ENABLED" : "DISABLED");
}

/* Attribute indices:
 *   [0] Primary Service, [1] Char Declaration, [2] Char VALUE, [3] CCC
 */
BT_GATT_SERVICE_DEFINE(vitalink_svc,
    BT_GATT_PRIMARY_SERVICE(&SVC_UUID.uuid),
    BT_GATT_CHARACTERISTIC(&CHR_UUID.uuid,
                           BT_GATT_CHRC_NOTIFY,
                           BT_GATT_PERM_NONE,
                           NULL, NULL, NULL),
    BT_GATT_CCC(ccc_cfg_changed, BT_GATT_PERM_READ | BT_GATT_PERM_WRITE)
);
#define VITALINK_CHAR_VALUE_ATTR (&vitalink_svc.attrs[2])

/* ===== Dynamic BLE name from identity address ============================ */
#define VITALINK_DYN_NAME_MAX 32
static void set_dynamic_name_from_addr(void)
{
    bt_addr_le_t addrs[CONFIG_BT_ID_MAX];
    size_t count = CONFIG_BT_ID_MAX;

    /* NCS 2.9.x: bt_id_get is void; it fills addrs[] and updates count. */
    bt_id_get(addrs, &count);
    if (count == 0) {
        LOG_WRN("No BLE identities; keeping default name");
        return;
    }

    const uint8_t *v = addrs[0].a.val;  /* 6 bytes */
    char dyn_name[VITALINK_DYN_NAME_MAX];
    snprintk(dyn_name, sizeof(dyn_name), "VitalLink-%02X%02X", v[1], v[0]);

    int err = bt_set_name(dyn_name);
    if (err) LOG_WRN("bt_set_name failed (%d)", err);
    else     LOG_INF("BLE name set to: %s", dyn_name);
}

/* ===== Advertising start (includes Service UUID) ========================= */
static void bt_ready_cb(int err)
{
    if (err) {
        LOG_ERR("Bluetooth init failed (%d)", err);
        return;
    }

    static const struct bt_data ad[] = {
        BT_DATA_BYTES(BT_DATA_FLAGS, (BT_LE_AD_GENERAL | BT_LE_AD_NO_BREDR)),
        VITALINK_AD_SERVICE_UUID,
    };

    err = bt_le_adv_start(BT_LE_ADV_CONN_NAME, ad, ARRAY_SIZE(ad), NULL, 0);
    if (err) LOG_ERR("Advertising start failed (%d)", err);
    else     LOG_INF("Advertising started");
}

/* ===== Notify helper ===================================================== */
static void send_vitals(const struct vitals_pkt *pkt)
{
    if (!notify_enabled) return;
    const struct bt_gatt_attr *attr = VITALINK_CHAR_VALUE_ATTR; /* value at index 2 */
    int err = bt_gatt_notify(NULL, attr, pkt, sizeof(*pkt));
    if (err) LOG_WRN("notify err %d", err);
}

/* ===== Tiering thresholds and policy ==================================== */
#define HR_ALERT_BPM        120
#define HR_EMERG_BPM        140
#define SPO2_ALERT_PCT       92
#define SPO2_EMERG_PCT       88
#define TEMP_ALERT_C_X100  3800   /* 38.00 C */
#define TEMP_EMERG_C_X100  3950   /* 39.50 C */

static uint8_t decide_flags_tier(uint8_t hr_bpm, uint8_t spo2, int16_t skin_c_x100)
{
    if (hr_bpm >= HR_EMERG_BPM || spo2 <= SPO2_EMERG_PCT || skin_c_x100 >= TEMP_EMERG_C_X100)
        return VFLAG_EMERGENCY;
    if (hr_bpm >= HR_ALERT_BPM  || spo2 <= SPO2_ALERT_PCT  || skin_c_x100 >= TEMP_ALERT_C_X100)
        return VFLAG_ALERT;
    return 0;
}

/* ===== Motion artifact flag (debounced on activity RMS) ================== */
/* Heuristic based on act_rms_x100 (units: 0.01 g). E.g., 400 => 4.00 m/s^2 ~ 0.4 g */
#define MA_HIGH_THRESH_X100   400  /* >=0.4 g RMS => motion likely corrupting PPG */
#define MA_SET_COUNT            2   /* N consecutive highs to set */
#define MA_CLR_COUNT            3   /* M consecutive lows to clear */
static uint8_t ma_high_run = 0, ma_low_run = 0;
static bool motion_artifact = false;

static void motion_artifact_update(uint16_t act_rms_x100)
{
    if (act_rms_x100 >= MA_HIGH_THRESH_X100) {
        ma_high_run++; ma_low_run = 0;
        if (!motion_artifact && ma_high_run >= MA_SET_COUNT) motion_artifact = true;
    } else {
        ma_low_run++; ma_high_run = 0;
        if (motion_artifact && ma_low_run >= MA_CLR_COUNT) motion_artifact = false;
    }
}

/* ===== Low battery flag (ADC if VBAT alias provided) ===================== */
/* Hysteresis to avoid flag flapping while charging.                          */
#define VBAT_LOW_MV        3000    /* trip threshold */
#define VBAT_RECOVER_MV    3120    /* clear threshold */

static int32_t vbat_read_mv(void)
{
#if HAS_VBAT_ADC
    if (!device_is_ready(vbat_adc.dev)) {
        LOG_WRN("VBAT ADC not ready");
        return -1;
    }

    /* Configure channel if needed; Zephyr helper handles lazy setup. */
    int err = adc_channel_setup_dt(&vbat_adc);
    if (err) {
        LOG_WRN("adc_channel_setup_dt failed (%d)", err);
        return -1;
    }

    int16_t sample = 0;
    struct adc_sequence seq = {
        .channels    = BIT(vbat_adc.channel_id),
        .buffer      = &sample,
        .buffer_size = sizeof(sample),
        .resolution  = vbat_adc.resolution,
        .oversampling = 0,
        .calibrate = false,
    };

    err = adc_read(vbat_adc.dev, &seq);
    if (err) {
        LOG_WRN("adc_read failed (%d)", err);
        return -1;
    }

    /* Convert raw to millivolts at ADC input */
    int32_t mv = sample;
    err = adc_raw_to_millivolts_dt(&vbat_adc, &mv);
    if (err) {
        LOG_WRN("adc_raw_to_millivolts_dt failed (%d)", err);
        return -1;
    }

    /* Scale to true VBAT if a divider is used:
     * NOTE: Provide your divider ratio via code or DT in the future.
     * For now assume the input already reflects VBAT (1:1) unless you change it.
     */
    return mv; /* if using a divider, multiply here: mv * (Rtop+Rbot)/Rbot */
#else
    /* No VBAT channel wired yet: report unavailable. */
    return -1;
#endif
}

static bool low_batt = false;
static void low_batt_update(void)
{
    int32_t mv = vbat_read_mv();
    if (mv < 0) return; /* not available yet; skip logic cleanly */

    if (mv <= VBAT_LOW_MV)          low_batt = true;
    else if (mv >= VBAT_RECOVER_MV) low_batt = false;
}

/* ===== Sensor fault flag (debounced failure counters) ==================== */
/* Trip when any sensor shows consecutive failures; clear when all recover.   */
#define SF_ERR_TRIP      3
#define SF_ERR_CLEAR     3
static uint8_t err_ppg = 0, ok_ppg = 0;
static uint8_t err_tmp = 0, ok_tmp = 0;
static uint8_t err_acc = 0, ok_acc = 0;
static bool sensor_fault = false;

static void sensor_fault_update(bool ppg_ok, bool tmp_ok, bool acc_ok)
{
    if (ppg_ok) { ok_ppg++; err_ppg = 0; } else { err_ppg++; ok_ppg = 0; }
    if (tmp_ok) { ok_tmp++; err_tmp = 0; } else { err_tmp++; ok_tmp = 0; }
    if (acc_ok) { ok_acc++; err_acc = 0; } else { err_acc++; ok_acc = 0; }

    bool any_trip = (err_ppg >= SF_ERR_TRIP) || (err_tmp >= SF_ERR_TRIP) || (err_acc >= SF_ERR_TRIP);
    if (any_trip) sensor_fault = true;
    else {
        if (ok_ppg >= SF_ERR_CLEAR && ok_tmp >= SF_ERR_CLEAR && ok_acc >= SF_ERR_CLEAR)
            sensor_fault = false;
    }
}

/* ===== Buzzer pulse (non-blocking) ======================================= */
#if HAS_BUZZER
#define BUZZER_ON_MS_EMERG  300
static struct k_work_delayable buzzer_off_work;
static void buzzer_off_fn(struct k_work *w) { ARG_UNUSED(w); gpio_pin_set_dt(&buzzer, 0); }
static void buzzer_pulse_emergency(void)
{
    if (!device_is_ready(buzzer.port)) return;
    gpio_pin_set_dt(&buzzer, 1);
    k_work_reschedule(&buzzer_off_work, K_MSEC(BUZZER_ON_MS_EMERG));
}
#else
static void buzzer_pulse_emergency(void) { /* no-op */ }
#endif

/* ===== Charging ⇒ System OFF (simple policy) ============================= */
/* If STAT (active-low) is LOW: immediately enter System OFF and wake on HIGH. */
#if HAS_CHG_STAT
#include <hal/nrf_gpio.h>
#include <hal/nrf_power.h>

static inline bool chg_is_active(void)
{
    int v = gpio_pin_get_dt(&chg_stat);
    return (v == 0); /* active-low: 0 => charging */
}

static void enter_system_off_with_wake_on_unplug(void)
{
    LOG_INF("Entering System OFF (charging)");
    /* For active-low STAT, wake on HIGH (unplug) */
    if (chg_stat.port == DEVICE_DT_GET(DT_NODELABEL(gpio0))) {
        nrf_gpio_cfg_sense_set(chg_stat.pin, NRF_GPIO_PIN_SENSE_HIGH);
    }
    nrf_power_system_off(NRF_POWER);
    /* execution halts here until wake; on wake, cold boot resumes at reset */
}

/* Boot-time check: power off immediately if already charging */
static void system_off_on_charge_boot_check(void)
{
    if (!device_is_ready(chg_stat.port)) {
        LOG_WRN("CHG_STAT not ready; skipping boot charge check");
        return;
    }
    if (gpio_pin_configure_dt(&chg_stat, GPIO_INPUT) != 0) {
        LOG_WRN("CHG_STAT config failed");
        return;
    }
    if (chg_is_active()) enter_system_off_with_wake_on_unplug();
}

/* Runtime detect: if charger plugged while running, power off quickly */
static struct gpio_callback chg_cb;
static struct k_work_delayable chg_debounced_off;
#define CHG_DEBOUNCE_MS 100

static void chg_debounced_off_fn(struct k_work *w)
{
    ARG_UNUSED(w);
    if (chg_is_active()) enter_system_off_with_wake_on_unplug();
}

static void chg_irq_handler(const struct device *dev, struct gpio_callback *cb, uint32_t pins)
{
    ARG_UNUSED(dev); ARG_UNUSED(cb); ARG_UNUSED(pins);
    /* Falling edge (HIGH->LOW) indicates "charging" for active-low STAT */
    k_work_reschedule(&chg_debounced_off, K_MSEC(CHG_DEBOUNCE_MS));
}
#endif /* HAS_CHG_STAT */

/* ===== 1 Hz work item (sampling + policy) ================================ */
static struct k_work_delayable tick_work;
static uint16_t seq;

#define NORMAL_HEARTBEAT_MS  (60 * 1000)
static uint32_t last_normal_sent_ms = 0;

static void tick_fn(struct k_work *work)
{
    ARG_UNUSED(work);

    /* Dummy "OK" status for now; wire these to real read results later. */
    bool ppg_ok = true, tmp_ok = true, acc_ok = true;

    /* Dummy vitals */
    uint8_t  hr_bpm       = 76;
    uint8_t  spo2         = 98;
    int16_t  skin_c_x100  = 3650;  /* 36.50 C */
    uint16_t act_rms_x100 = 180;   /* 1.80 * 0.01 g units */

    /* ---- Derived flags upkeep ----------------------------------------- */
    motion_artifact_update(act_rms_x100);
    low_batt_update();
    sensor_fault_update(ppg_ok, tmp_ok, acc_ok);

    /* ---- Tier (alert/emergency) --------------------------------------- */
    uint8_t flags = decide_flags_tier(hr_bpm, spo2, skin_c_x100);
    if (motion_artifact) flags |= VFLAG_MOTION_ARTIFACT;
    if (low_batt)        flags |= VFLAG_LOW_BATT;
    if (sensor_fault)    flags |= VFLAG_SENSOR_FAULT;

    /* ---- Build packet -------------------------------------------------- */
    struct vitals_pkt p = {0};
    p.seq   = seq++;
    p.ts_ms = k_uptime_get_32();
    p.flags = flags;
    p.hr_bpm       = hr_bpm;
    p.spo2         = spo2;
    p.skin_c_x100  = skin_c_x100;
    p.act_rms_x100 = act_rms_x100;
    vitals_finalize(&p);  /* ver, rfu, checksum */

    /* ---- Send policy --------------------------------------------------- */
    bool should_send = false;

    if ((flags & (VFLAG_ALERT | VFLAG_EMERGENCY)) != 0) {
        should_send = true;
        if (flags & VFLAG_EMERGENCY) buzzer_pulse_emergency();
    } else {
        uint32_t now = p.ts_ms;
        if ((now - last_normal_sent_ms) >= NORMAL_HEARTBEAT_MS) {
            should_send = true;
            last_normal_sent_ms = now;
        }
    }

    if (should_send) {
        /* simple log preview of what we send */
        LOG_INF("HR=%u BPM, SpO2=%u%%, Temp=%.2fC, flags=0x%02x",
                p.hr_bpm, p.spo2, p.skin_c_x100 / 100.0f, p.flags);
        send_vitals(&p);
    }

    /* Reschedule for 1 Hz */
    k_work_reschedule(&tick_work, K_SECONDS(1));
}

/* ===== App entry ========================================================= */
void main(void)
{
    /* I2C device handle (for later sensor work) */
    i2c_dev = DEVICE_DT_GET(I2C_NODE);
    if (!device_is_ready(i2c_dev)) {
        LOG_WRN("I2C device not ready (OK for now)");
    }

#if HAS_BUZZER
    if (device_is_ready(buzzer.port)) {
        int err = gpio_pin_configure_dt(&buzzer, GPIO_OUTPUT_INACTIVE);
        if (err) LOG_WRN("Buzzer gpio config failed (%d)", err);
        k_work_init_delayable(&buzzer_off_work, buzzer_off_fn);
    } else {
        LOG_WRN("Buzzer GPIO not ready");
    }
#endif

#if HAS_CHG_STAT
    /* Boot-time "charging ⇒ System OFF" check BEFORE starting BLE, etc. */
    system_off_on_charge_boot_check();

    /* Runtime detect: configure interrupt so plugging-in powers off quickly */
    if (device_is_ready(chg_stat.port)) {
        gpio_init_callback(&chg_cb, chg_irq_handler, BIT(chg_stat.pin));
        gpio_add_callback(chg_stat.port, &chg_cb);
        /* Falling edge (HIGH->LOW) for active-low STAT = start charging */
        gpio_pin_interrupt_configure_dt(&chg_stat, GPIO_INT_EDGE_FALLING);
        k_work_init_delayable(&chg_debounced_off, chg_debounced_off_fn);
    } else {
        LOG_WRN("CHG_STAT GPIO not ready for runtime detect");
    }
#endif

    /* Dynamic BLE name BEFORE advertising */
    set_dynamic_name_from_addr();

    /* Start Bluetooth stack; advertising begins in callback on success */
    int err = bt_enable(bt_ready_cb);
    if (err) {
        LOG_ERR("bt_enable failed: %d", err);
        return;
    }

    /* Start 1 Hz sampler/publisher */
    k_work_init_delayable(&tick_work, tick_fn);
    k_work_schedule(&tick_work, K_SECONDS(1));
}

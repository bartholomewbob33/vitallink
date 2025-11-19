/* -----------------------------------------------------------------------------
 * File: src/main.c
 * Purpose: VitalLink wristband firmware (nRF5340 DK, NCS v2.9.2)
 *
 * Key features:
 *  - BLE Peripheral with dynamic device name "VitalLink-XXXX"
 *  - Custom GATT service + NOTIFY characteristic (UUIDs in vitalink_uuids.h)
 *  - 1 Hz vitals sampling via sensors_vitalink (raw I2C for MAX30102/LSM6DSOX/MAX30208),
 *    tiering, checksum
 *  - BATTERY + CHARGING via BQ25155 over I2C:
 *      * Low-battery flag injected into packet flags
 *      * "Docked idle" while charging: stop BLE/sampling, poll PMIC every 3 s,
 *        resume automatically when unplugged
 *  - Optional buzzer (P1.04 active-high) on EMERGENCY tier
 *  - I2C1 on P0.19 (SCL) / P0.21 (SDA) for sensors + PMIC
 * --------------------------------------------------------------------------- */

#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <zephyr/drivers/i2c.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/bluetooth/bluetooth.h>
#include <zephyr/bluetooth/gatt.h>
#include <zephyr/logging/log.h>
#include <stdbool.h>
#include <string.h>

#include "sensors_vitalink.h"
#include "vitalink_uuids.h"
#include "vitalink_proto.h"
#include "pmic_bq25155.h"

LOG_MODULE_REGISTER(vitalink, LOG_LEVEL_INF);

/* ===== I2C handle (sensors + PMIC) ======================================= */
/* Overlay alias is 'i2c-ppg'; Zephyr macro uses underscore form 'i2c_ppg' */
#define I2C_NODE DT_ALIAS(i2c_ppg)
static const struct device *i2c0_dev;

/* ===== Optional buzzer (active-high) ===================================== */
#if DT_NODE_HAS_STATUS(DT_ALIAS(buzzer), okay)
#define HAS_BUZZER 1
static const struct gpio_dt_spec buzzer = GPIO_DT_SPEC_GET(DT_ALIAS(buzzer), gpios);
#else
#define HAS_BUZZER 0
#endif

/* ===== Aliased GPIOs for INTs and charge enable ========================== */
static const struct gpio_dt_spec pin_temp_int = GPIO_DT_SPEC_GET(DT_ALIAS(temp_int), gpios);
static const struct gpio_dt_spec pin_hr_int   = GPIO_DT_SPEC_GET(DT_ALIAS(hr_int),   gpios);
static const struct gpio_dt_spec pin_imu_int  = GPIO_DT_SPEC_GET(DT_ALIAS(imu_int),  gpios);
static const struct gpio_dt_spec pin_chg_en   = GPIO_DT_SPEC_GET(DT_ALIAS(chg_en),   gpios);

/* ===== BLE UUIDs ========================================================= */
static struct bt_uuid_128 svc_uuid = VITALINK_SERVICE_UUID_INIT;
static struct bt_uuid_128 chr_uuid = VITALS_CHAR_UUID_INIT;

/* CCC subscription flag */
static uint8_t notify_enabled;

/* OPTIONAL: track connection so notify targets the active central */
static struct bt_conn *current_conn;

/* ===== Battery thresholds (mV) =========================================== */
#define LOW_BATT_ALERT_MV    3500
#define LOW_BATT_RECOVER_MV  3600

/* ===== Tier thresholds (defaults; refine clinically later) =============== */
#define HR_ALERT_BPM        120
#define HR_EMERG_BPM        140
#define SPO2_ALERT_PCT       92
#define SPO2_EMERG_PCT       88
#define TEMP_ALERT_C_X100  3800   /* 38.00 C */
#define TEMP_EMERG_C_X100  3950   /* 39.50 C */

#define BUZZER_ON_MS_EMERG  300

/* "extreme motion" (possible fall)          */
/* act_rms_x100 is RMS acceleration in g * 100 (≈200 at 2 g full-scale).    */
#define FALL_ACCEL_THRESHOLD_X100  180   /* ~1.8 g – tweak based on testing */

/* ===== App state ========================================================= */
enum run_state { RUN_NORMAL = 0, DOCKED_IDLE = 1 };
static enum run_state pwr_state = RUN_NORMAL;
static bool low_batt_flag = false;

static struct k_work_delayable tick_work;   /* 1 Hz vitals */
static struct k_work_delayable pmic_work;   /* 3 s PMIC poll */
static uint16_t seq;

/* ===== GATT service/CCC ================================================== */
static void ccc_cfg_changed(const struct bt_gatt_attr *attr, uint16_t value)
{
	notify_enabled = (value == BT_GATT_CCC_NOTIFY);
	LOG_INF("Notify %s", notify_enabled ? "ENABLED" : "DISABLED");
}

/* Attr idx:
 * [0] Primary Service
 * [1] Char Declaration
 * [2] Char VALUE (notify targets this)
 * [3] CCC
 */
BT_GATT_SERVICE_DEFINE(vitalink_svc,
	BT_GATT_PRIMARY_SERVICE(&svc_uuid),
	BT_GATT_CHARACTERISTIC(&chr_uuid.uuid,
		BT_GATT_CHRC_NOTIFY,
		BT_GATT_PERM_NONE,
		NULL, NULL, NULL),
	BT_GATT_CCC(ccc_cfg_changed, BT_GATT_PERM_READ | BT_GATT_PERM_WRITE)
);

/* ===== Connection callbacks ============================================== */
static void connected(struct bt_conn *conn, uint8_t err)
{
	if (err) {
		LOG_WRN("Connected failed (err %u)", err);
		return;
	}
	current_conn = bt_conn_ref(conn);
	LOG_INF("Connected");
}

static void disconnected(struct bt_conn *conn, uint8_t reason)
{
	ARG_UNUSED(conn);
	LOG_INF("Disconnected (reason %u)", reason);
	if (current_conn) {
		bt_conn_unref(current_conn);
		current_conn = NULL;
	}
}

BT_CONN_CB_DEFINE(conn_cb) = {
	.connected = connected,
	.disconnected = disconnected,
};

/* ===== BLE name helper =================================================== */
#define VITALINK_DYN_NAME_MAX 32
static void set_dynamic_name_from_addr(void)
{
	bt_addr_le_t addrs[CONFIG_BT_ID_MAX];
	size_t count = CONFIG_BT_ID_MAX;
	bt_id_get(addrs, &count);

	if (count == 0) {
		LOG_WRN("No BLE identities found; keeping default name");
		return;
	}
	const uint8_t *v = addrs[0].a.val; /* 6 bytes, LE */
	char dyn_name[VITALINK_DYN_NAME_MAX];
	snprintk(dyn_name, sizeof(dyn_name), "VitalLink-%02X%02X", v[1], v[0]);
	int err = bt_set_name(dyn_name);
	if (err) LOG_WRN("bt_set_name failed (%d)", err);
	else     LOG_INF("BLE name set to: %s", dyn_name);
}

static int start_advertising(void)
{
	static const struct bt_le_adv_param adv = BT_LE_ADV_PARAM_INIT(
		BT_LE_ADV_OPT_CONNECTABLE | BT_LE_ADV_OPT_USE_NAME,
		BT_GAP_ADV_FAST_INT_MIN_2,
		BT_GAP_ADV_FAST_INT_MAX_2,
		NULL
	);
	return bt_le_adv_start(&adv, NULL, 0, NULL, 0);
}

/* ===== Checksum helper (8-bit) ========================================== */
static uint8_t checksum8(const uint8_t *p, size_t n)
{
	uint32_t sum = 0;
	for (size_t i = 0; i < n; i++) sum += p[i];
	return (uint8_t)(sum & 0xFF);
}

/* ===== Tier logic ======================================================== */
static uint8_t decide_flags(uint8_t hr_bpm, uint8_t spo2, int16_t skin_c_x100)
{
	if (hr_bpm >= HR_EMERG_BPM || spo2 <= SPO2_EMERG_PCT || skin_c_x100 >= TEMP_EMERG_C_X100) {
		return VFLAG_EMERGENCY;
	}
	if (hr_bpm >= HR_ALERT_BPM || spo2 <= SPO2_ALERT_PCT || skin_c_x100 >= TEMP_ALERT_C_X100) {
		return VFLAG_ALERT;
	}
	return 0;
}

static inline uint8_t apply_system_flags(uint8_t base_flags)
{
	if (low_batt_flag) base_flags |= VFLAG_LOW_BATT;
	return base_flags;
}

static void buzzer_pulse_emergency(void)
{
#if HAS_BUZZER
	if (!device_is_ready(buzzer.port)) return;
	gpio_pin_set_dt(&buzzer, 1);
	k_sleep(K_MSEC(BUZZER_ON_MS_EMERG));
	gpio_pin_set_dt(&buzzer, 0);
#endif
}

/* ===== Notify helper ===================================================== */
static void send_vitals(const struct vitals_pkt *pkt)
{
	if (!notify_enabled) return;
	const struct bt_gatt_attr *attr = &vitalink_svc.attrs[2]; /* value attr */
	int err = bt_gatt_notify(current_conn, attr, pkt, sizeof(*pkt));
	if (err && err != -ENOTCONN) {
		err = bt_gatt_notify(NULL, attr, pkt, sizeof(*pkt));
	}
	if (err) LOG_WRN("notify err %d", err);
}

/* ===== Sampling policy =================================================== */
#define NORMAL_HEARTBEAT_MS (60 * 1000)
static uint32_t last_normal_sent_ms = 0;

/* ===== 1 Hz vitals tick ================================================== */
static void tick_fn(struct k_work *work)
{
	ARG_UNUSED(work);

	/* Keep last-good values to bridge noisy windows */
	static uint8_t  last_hr_bpm       = 76;
	static uint8_t  last_spo2         = 98;
	static int16_t  last_skin_c_x100  = 3650;
	static uint16_t last_act_rms_x100 = 180;

	uint8_t  hr_bpm;
	uint8_t  spo2;
	int16_t  skin_c_x100;
	uint16_t act_rms_x100;

	/* Single facade call now does everything via raw I2C */
	int s_ok = sensors_read(&hr_bpm, &spo2, &skin_c_x100, &act_rms_x100);
	if (s_ok < 0) {
		hr_bpm       = last_hr_bpm;
		spo2         = last_spo2;
		skin_c_x100  = last_skin_c_x100;
		act_rms_x100 = last_act_rms_x100;
	} else {
		last_hr_bpm       = hr_bpm;
		last_spo2         = spo2;
		last_skin_c_x100  = skin_c_x100;
		last_act_rms_x100 = act_rms_x100;
	}

    if (spo2 > 100) spo2 = 100;

    /* Base tier decision from HR / SpO2 / Temp */
    uint8_t flags = decide_flags(hr_bpm, spo2, skin_c_x100);

    /* Extreme motion (possible fall): treat as EMERGENCY as well */
    bool fall_like_motion = (act_rms_x100 >= FALL_ACCEL_THRESHOLD_X100);
    if (fall_like_motion) {
        flags |= VFLAG_EMERGENCY;
		LOG_WRN("Extreme motion detected (act_rms_x100=%u) -> EMERGENCY", act_rms_x100);
    }
	

    /* Add system-level flags (low battery, etc.) */
    flags = apply_system_flags(flags);

	struct vitals_pkt p = {0};
	p.ver   = VITALINK_PROTO_VER;
	p.seq   = seq++;
	p.ts_ms = k_uptime_get_32();
	p.flags = flags;
	p.hr_bpm       = hr_bpm;
	p.spo2         = spo2;
	p.skin_c_x100  = skin_c_x100;
	p.act_rms_x100 = act_rms_x100;
	p.checksum     = checksum8((const uint8_t *)&p, 14);

	bool should_send = false;
	if (flags & (VFLAG_ALERT | VFLAG_EMERGENCY)) {
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
		send_vitals(&p);
		LOG_INF("HR=%u BPM, SpO2=%u%% %s%s",
		        p.hr_bpm, p.spo2,
		        (flags & VFLAG_LOW_BATT) ? "[LOW_BATT] " : "",
		        (flags & VFLAG_EMERGENCY) ? "[EMERG]" :
		        (flags & VFLAG_ALERT) ? "[ALERT]" : "");
	}

	k_work_schedule(&tick_work, K_SECONDS(1));
}

/* ===== PMIC polling: low-batt + docked-idle ============================== */
static void enter_docked_idle(void)
{
	if (pwr_state == DOCKED_IDLE) return;
	bt_le_adv_stop();
	k_work_cancel_delayable(&tick_work);
	pwr_state = DOCKED_IDLE;
	LOG_INF("Docked: charging → pause BLE/sampling");
}

static void exit_docked_idle(void)
{
	if (pwr_state == RUN_NORMAL) return;
	int err = start_advertising();
	if (err) LOG_WRN("adv restart err %d", err);
	k_work_schedule(&tick_work, K_SECONDS(1));
	pwr_state = RUN_NORMAL;
	LOG_INF("Undocked: resume BLE/sampling");
}

static void pmic_fn(struct k_work *work)
{
	ARG_UNUSED(work);
	bool usb_ok=false, charging=false;
	uint16_t vbat_mv=0;

	int s1 = bq25155_read_status(i2c0_dev, &usb_ok, &charging);
	int s2 = bq25155_read_vbat_mv(i2c0_dev, &vbat_mv);

	if (s2 == 0) {
		if (!low_batt_flag && vbat_mv <= LOW_BATT_ALERT_MV) {
			low_batt_flag = true;
			LOG_WRN("Low battery: %u mV", vbat_mv);
		} else if (low_batt_flag && vbat_mv >= LOW_BATT_RECOVER_MV) {
			low_batt_flag = false;
			LOG_INF("Battery recovered: %u mV", vbat_mv);
		}
	}

	if (s1 == 0) {
		if (charging && pwr_state == RUN_NORMAL) {
			enter_docked_idle();
		} else if (!charging && pwr_state == DOCKED_IDLE) {
			exit_docked_idle();
		}
	}

	k_work_schedule(&pmic_work, K_SECONDS(3));
}

/* ===== BLE ready callback ================================================ */
static void bt_ready_cb(int err)
{
	if (err) {
		LOG_ERR("Bluetooth init failed (%d)", err);
		return;
	}
	err = start_advertising();
	if (err) LOG_ERR("Advertising start failed (%d)", err);
	else     LOG_INF("Advertising started");
}

/* ===== App entry ========================================================= */
void main(void)
{
	/* I2C dev */
	i2c0_dev = DEVICE_DT_GET(I2C_NODE);

	if (!device_is_ready(i2c0_dev)) {
		LOG_ERR("I2C not ready");
	} else {
		/* PMIC init */
		(void)bq25155_init(i2c0_dev);

		/* Initialize sensor facade (raw I2C paths) */
		int sret = sensors_init(i2c0_dev);
		if (sret) {
			LOG_WRN("sensors_init returned %d (falling back to last-good if needed)", sret);
		}

		/* If already charging at boot, enter DOCKED_IDLE right away */
		bool usb_ok = false, charging = false;
		if (bq25155_read_status(i2c0_dev, &usb_ok, &charging) == 0) {
			if (charging && pwr_state == RUN_NORMAL) {
				enter_docked_idle();  /* stops adv + sampling */
			}
		}
	}

	/* Buzzer */
#if HAS_BUZZER
	if (device_is_ready(buzzer.port)) {
		int e = gpio_pin_configure_dt(&buzzer, GPIO_OUTPUT_INACTIVE);
		if (e) { LOG_WRN("buzzer gpio cfg err %d", e); }
	}
#endif

	/* INT inputs + charge enable default (enable charger: CE_N low) */
	if (device_is_ready(pin_temp_int.port)) {
		int e = gpio_pin_configure_dt(&pin_temp_int, GPIO_INPUT);
		if (e) { LOG_WRN("temp_int cfg err %d", e); }
	}
	if (device_is_ready(pin_hr_int.port)) {
		int e = gpio_pin_configure_dt(&pin_hr_int, GPIO_INPUT);
		if (e) { LOG_WRN("hr_int cfg err %d", e); }
	}
	if (device_is_ready(pin_imu_int.port)) {
		int e = gpio_pin_configure_dt(&pin_imu_int, GPIO_INPUT);
		if (e) { LOG_WRN("imu_int cfg err %d", e); }
	}
	if (device_is_ready(pin_chg_en.port)) {
		int e = gpio_pin_configure_dt(&pin_chg_en, GPIO_OUTPUT_ACTIVE);
		if (e) { LOG_WRN("chg_en cfg err %d", e); }
	}

	/* Dynamic name before advertising */
	set_dynamic_name_from_addr();

	/* Start BLE unless we already entered DOCKED_IDLE above */
	int err = bt_enable(bt_ready_cb);
	if (err) {
		LOG_ERR("bt_enable failed: %d", err);
		return;
	}
	if (pwr_state == DOCKED_IDLE) {
		bt_le_adv_stop();
		LOG_INF("Booted in docked state: BLE/sampling paused");
	}

	/* start 1 Hz sampler/publisher (only in RUN_NORMAL) */
	k_work_init_delayable(&tick_work, tick_fn);
	if (pwr_state == RUN_NORMAL) {
		k_work_schedule(&tick_work, K_SECONDS(1));
	}

	/* start PMIC polling (low-batt + docked-idle) */
	k_work_init_delayable(&pmic_work, pmic_fn);
	k_work_schedule(&pmic_work, K_SECONDS(3));
}

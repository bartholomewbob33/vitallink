/* -----------------------------------------------------------------------------
 * File: src/sensors_vitalink.c
 * Purpose: VitalLink sensors over raw I2C + lightweight HR/SpO2 engine
 *
 * - MAX30208 (skin temp): raw I2C (0x4A) → °C * 100
 * - MAX30102 (PPG): raw I2C (0x57)
 *      * Configure SPO2 mode, 100 Hz, 411 us PW, ~12 mA LEDs
 *      * 50 Hz internal sampler (delayable work) reads FIFO and stores IR/RED
 *      * 4 s window analysis → HR (BPM) + SpO2 (%)
 * - LSM6DSOX (IMU @0x6A): optional raw accel read → RMS * 100
 *
 * No Zephyr sensor drivers are required.
 * --------------------------------------------------------------------------- */

#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <zephyr/drivers/i2c.h>
#include <zephyr/logging/log.h>
#include <math.h>
#include <string.h>
#include <stdlib.h>
#include "sensors_vitalink.h"

LOG_MODULE_REGISTER(vl_sensors, LOG_LEVEL_INF);

/* ===== I2C bus from devicetree alias i2c-ppg ============================= */
#define I2C_NODE DT_ALIAS(i2c_ppg)
static const struct device *g_i2c = DEVICE_DT_GET(I2C_NODE);

/* Small, unambiguous helpers that DO NOT shadow Zephyr APIs */
static int i2c_reg_write_u8(uint8_t addr, uint8_t reg, uint8_t val)
{
    uint8_t tx[2] = { reg, val };
    return i2c_write(g_i2c, tx, 2, addr);
}
static int i2c_reg_read_u8(uint8_t addr, uint8_t reg, uint8_t *val)
{
    return i2c_write_read(g_i2c, addr, &reg, 1, val, 1);
}
static int i2c_read_multi(uint8_t addr, uint8_t start_reg, uint8_t *buf, size_t len)
{
    return i2c_write_read(g_i2c, addr, &start_reg, 1, buf, len);
}

/* ===== MAX30208 (skin temp) ============================================== */
#define MAX30208_ADDR          0x50  
#define MAX30208_REG_TEMP_MSB  0x02

static int max30208_read_temp_x100(int16_t *out_c_x100)
{
    if (!device_is_ready(g_i2c) || !out_c_x100) return -EINVAL;

    uint8_t reg = MAX30208_REG_TEMP_MSB;
    uint8_t buf[2] = {0};
    int err = i2c_write_read(g_i2c, MAX30208_ADDR, &reg, 1, buf, 2);
    if (err) return err;

    int16_t raw = (int16_t)((buf[0] << 8) | buf[1]);   /* signed */
    int32_t cx100 = (int32_t)raw * 100 / 256;         /* °C * 100 */
    *out_c_x100 = (int16_t)cx100;
    return 0;
}

/* ===== LSM6DSOX (IMU) minimal raw accel ================================ */
#define LSM6_ADDR           0x6A
#define LSM6_REG_WHOAMI     0x0F
#define LSM6_REG_CTRL1_XL   0x10
#define LSM6_REG_OUTX_L_A   0x28

static int lsm6_init_minimal(void)
{
    if (!device_is_ready(g_i2c)) return -ENODEV;
    uint8_t who=0;
    (void)i2c_reg_read_u8(LSM6_ADDR, LSM6_REG_WHOAMI, &who);
    /* CTRL1_XL: 0b0100 1000 → ODR=104Hz (0100), FS=2g (00), BW=100 (10) */
    return i2c_reg_write_u8(LSM6_ADDR, LSM6_REG_CTRL1_XL, 0x48);
}

/* Converts raw ±2g 16-bit (rough scale for demo) */
static int lsm6_activity_rms_x100(uint16_t *out_rms_x100)
{
    if (!device_is_ready(g_i2c) || !out_rms_x100) return -EINVAL;
    uint8_t buf[6]={0};
    int err = i2c_read_multi(LSM6_ADDR, LSM6_REG_OUTX_L_A, buf, 6);
    if (err) return err;

    int16_t rx = (int16_t)(buf[1] << 8 | buf[0]);
    int16_t ry = (int16_t)(buf[3] << 8 | buf[2]);
    int16_t rz = (int16_t)(buf[5] << 8 | buf[4]);

    const float lsb_per_g = 16384.0f; /* rough */
    float x = rx / lsb_per_g, y = ry / lsb_per_g, z = rz / lsb_per_g;
    float rms = sqrtf(x*x + y*y + z*z);
    uint32_t q = (uint32_t)(rms * 100.0f);
    *out_rms_x100 = (q > 0xFFFFu) ? 0xFFFFu : (uint16_t)q;
    return 0;
}

/* ===== MAX30102 (PPG) raw I2C + tiny HR/SpO2 engine ===================== */
#define MAX30102_ADDR       0x57

/* Registers */
#define REG_INT_STATUS1     0x00
#define REG_INT_STATUS2     0x01
#define REG_INT_ENABLE1     0x02
#define REG_INT_ENABLE2     0x03
#define REG_FIFO_WR_PTR     0x04
#define REG_OVF_COUNTER     0x05
#define REG_FIFO_RD_PTR     0x06
#define REG_FIFO_DATA       0x07
#define REG_FIFO_CONFIG     0x08
#define REG_MODE_CONFIG     0x09
#define REG_SPO2_CONFIG     0x0A
#define REG_LED1_PA         0x0C
#define REG_LED2_PA         0x0D
#define REG_MULTILED_CTRL1  0x11
#define REG_MULTILED_CTRL2  0x12
#define REG_PART_ID         0xFF

/* Sampler timing */
#define PPG_SPS_HZ          50
#define PPG_DT_MS           (1000/PPG_SPS_HZ)
#define PPG_WIN_SEC         4
#define PPG_WIN_LEN         (PPG_SPS_HZ * PPG_WIN_SEC)   /* 200 */
#define PPG_MIN_HR_BPM      30
#define PPG_MAX_HR_BPM      220
#define PPG_MIN_BEATS       2

/* Ring buffer for last 4 s */
static uint32_t ring_ir[PPG_WIN_LEN];
static uint32_t ring_red[PPG_WIN_LEN];
static uint16_t ring_idx;
static bool     ring_filled;

/* latest estimates (debounced) */
static uint8_t g_hr_bpm = 76;
static uint8_t g_spo2   = 98;
static bool    g_hr_ok  = false;
static bool    g_s2_ok  = false;

/* NEW: scratch buffers for analysis (avoid large stack usage) */
static uint32_t ppg_ir_win[PPG_WIN_LEN];
static uint32_t ppg_red_win[PPG_WIN_LEN];

static struct k_work_delayable ppg_work;

static inline uint32_t clamp_u32(uint32_t v, uint32_t lo, uint32_t hi) {
    return (v < lo) ? lo : (v > hi ? hi : v);
}

/* small helpers */
static void ring_push(uint32_t ir, uint32_t red)
{
    ring_ir[ring_idx]  = ir;
    ring_red[ring_idx] = red;
    ring_idx = (ring_idx + 1) % PPG_WIN_LEN;
    if (ring_idx == 0) ring_filled = true;
}

static int copy_window(uint32_t *ir_out, uint32_t *red_out)
{
    if (!ring_filled && ring_idx < PPG_SPS_HZ*2) return -EAGAIN;
    const uint16_t start = ring_filled ? ring_idx : 0;
    const uint16_t n     = ring_filled ? PPG_WIN_LEN : ring_idx;
    if (!n) return -EAGAIN;

    if (ring_filled) {
        const uint16_t first = PPG_WIN_LEN - start;
        memcpy(ir_out,  &ring_ir[start],  first * sizeof(uint32_t));
        memcpy(red_out, &ring_red[start], first * sizeof(uint32_t));
        memcpy(ir_out + first,  &ring_ir[0],  start * sizeof(uint32_t));
        memcpy(red_out + first, &ring_red[0], start * sizeof(uint32_t));
    } else {
        memcpy(ir_out,  ring_ir,  n * sizeof(uint32_t));
        memcpy(red_out, ring_red, n * sizeof(uint32_t));
    }
    return n;
}

/* tiny utilities */
static void isort_int(int *a, int n) {
    for (int i=1;i<n;i++){int t=a[i],j=i-1;while(j>=0&&a[j]>t){a[j+1]=a[j];j--;}a[j+1]=t;}
}
static int median_int(const int *a, int n) {
    if (n<=0) return 0;
    int tmp[8]; int m = (n>8)?8:n;
    for (int i=0;i<m;i++) tmp[i]=a[i];
    isort_int(tmp,m);
    return (m&1)? tmp[m/2] : (tmp[m/2-1]+tmp[m/2])/2;
}
static int find_peaks(const int32_t *x, int n, int min_height, int min_dist, int *peaks, int maxp)
{
    int np=0;
    for (int i=1;i<n-1;i++) {
        if (x[i] > min_height && x[i] > x[i-1] && x[i] >= x[i+1]) {
            bool too_close=false;
            for (int j=0;j<np;j++){
                if (abs(peaks[j]-i) < min_dist) {
                    if (x[i] > x[peaks[j]]) peaks[j] = i;
                    too_close = true;
                    break;
                }
            }
            if (!too_close && np < maxp) peaks[np++] = i;
        }
    }
    isort_int(peaks, np);
    return np;
}

/* PPG analysis (IR/RED → HR/SpO2) */
static int analyze_ppg(const uint32_t *ir_u, const uint32_t *red_u, int n,
                       uint8_t *hr_bpm, bool *hr_ok, uint8_t *spo2_pct, bool *spo2_ok)
{
    if (n < PPG_SPS_HZ*2) return -EAGAIN;

    static int32_t ir[PPG_WIN_LEN], red[PPG_WIN_LEN];
    int64_t sum_ir=0,sum_red=0;
    for (int i=0;i<n;i++){sum_ir+=ir_u[i];sum_red+=red_u[i];}
    int32_t mean_ir=(int32_t)(sum_ir/n), mean_red=(int32_t)(sum_red/n);

    for (int i=0;i<n;i++){ ir[i]=(int32_t)ir_u[i]-mean_ir; red[i]=(int32_t)red_u[i]-mean_red; }
    for (int i=0;i<n-3;i++){ ir[i]=(ir[i]+ir[i+1]+ir[i+2]+ir[i+3])/4;
                             red[i]=(red[i]+red[i+1]+red[i+2]+red[i+3])/4; }

    static int32_t inv_ir[PPG_WIN_LEN];
    for (int i=0;i<n;i++) inv_ir[i] = -ir[i];

    int64_t abs_acc=0; for (int i=0;i<n;i++) abs_acc += (inv_ir[i]>=0?inv_ir[i]:-inv_ir[i]);
    int min_height = (int)(abs_acc/n);
    if (min_height < 10) min_height = 10;
    if (min_height > 2000) min_height = 2000;
    int min_dist = (int)(PPG_SPS_HZ*0.3f);

    int peaks[16]; int np = find_peaks(inv_ir, n, min_height, min_dist, peaks, 16);

    *hr_ok=false; *spo2_ok=false;

    /* HR from inter-peak */
    if (np >= PPG_MIN_BEATS+1) {
        int intervals[16], ni=0;
        for (int i=1;i<np;i++){int d=peaks[i]-peaks[i-1]; if (d>0) intervals[ni++]=d;}
        if (ni>0){
            double mean_samp=0.0; for (int i=0;i<ni;i++) mean_samp+=intervals[i];
            mean_samp/=ni;
            double bpm=60.0*(double)PPG_SPS_HZ/mean_samp;
            if (bpm>=PPG_MIN_HR_BPM && bpm<=PPG_MAX_HR_BPM) { *hr_bpm=(uint8_t)lround(bpm); *hr_ok=true; }
        }
    }

    /* SpO2 via ratio-of-ratios, median over beats (demo mapping) */
    if (np >= PPG_MIN_BEATS+1) {
        int ratios_q[8], nr=0;
        for (int k=0;k<np-1 && nr<8;k++){
            int a=peaks[k], b=peaks[k+1]; if (b-a<min_dist) continue;
            int32_t ir_max = INT32_MIN, red_max=INT32_MIN;
            double ir_rms=0, red_rms=0; int len=b-a; if (len<4) continue;
            for (int i=a;i<b;i++){ if (ir[i]>ir_max) ir_max=ir[i]; if (red[i]>red_max) red_max=red[i]; }
            for (int i=a;i<b;i++){ ir_rms += (double)ir[i]*ir[i]; red_rms += (double)red[i]*red[i]; }
            ir_rms = sqrt(ir_rms/len); red_rms = sqrt(red_rms/len);
            if (ir_max==0||red_max==0||ir_rms<=1e-3||red_rms<=1e-3) continue;
            double R = (red_rms/(double)red_max)/(ir_rms/(double)ir_max);
            int Rq = (int)lround(R*100.0);
            if (Rq>0 && Rq<300) ratios_q[nr++]=Rq;
        }
        if (nr>0){
            int Rmed = median_int(ratios_q,nr);
            double R = Rmed/100.0;
            double s2 = 110.0 - 25.0*R;      /* demo fit */
            if (s2<70.0) s2=70.0; if (s2>100.0) s2=100.0;
            *spo2_pct=(uint8_t)lround(s2); *spo2_ok=true;
        }
    }

    return (*hr_ok || *spo2_ok) ? 0 : -EAGAIN;
}

/* Configure MAX30102 for RED+IR in SPO2 mode, 100 Hz, PW=411us, ~12mA LEDs */
static int max30102_init_config(void)
{
    if (!device_is_ready(g_i2c)) return -ENODEV;

    /* Soft reset */
    (void)i2c_reg_write_u8(MAX30102_ADDR, REG_MODE_CONFIG, 0x40);
    k_sleep(K_MSEC(10));

    /* Clear FIFO pointers */
    (void)i2c_reg_write_u8(MAX30102_ADDR, REG_FIFO_WR_PTR, 0x00);
    (void)i2c_reg_write_u8(MAX30102_ADDR, REG_OVF_COUNTER, 0x00);
    (void)i2c_reg_write_u8(MAX30102_ADDR, REG_FIFO_RD_PTR, 0x00);

    /* FIFO Config: sample avg = 4 (0b010 <<5), FIFO rollover off, almost full = 0x0F */
    (void)i2c_reg_write_u8(MAX30102_ADDR, REG_FIFO_CONFIG, 0x4F);

    /* SPO2 Config:
       [7:5] SPO2_ADC_RGE=0 (2048nA), [4:2] SR=0b011 (100Hz), [1:0] PW=0b11 (411us, 18-bit) */
    (void)i2c_reg_write_u8(MAX30102_ADDR, REG_SPO2_CONFIG, 0x2B);

    /* LED1/2 current: ~12 mA (0x24 ≈ 12.6mA typical, adjust per board) */
    (void)i2c_reg_write_u8(MAX30102_ADDR, REG_LED1_PA, 0x24);  /* RED */
    (void)i2c_reg_write_u8(MAX30102_ADDR, REG_LED2_PA, 0x24);  /* IR  */

    /* Multi-LED slots: slot1=RED (0x01), slot2=IR (0x02) */
    (void)i2c_reg_write_u8(MAX30102_ADDR, REG_MULTILED_CTRL1, 0x21);
    (void)i2c_reg_write_u8(MAX30102_ADDR, REG_MULTILED_CTRL2, 0x00);

    /* Mode: SPO2 mode (0x03) for RED+IR */
    (void)i2c_reg_write_u8(MAX30102_ADDR, REG_MODE_CONFIG, 0x03);

    return 0;
}

/* Read one RED+IR sample from FIFO (6 bytes: RED[23:0], IR[23:0]) */
static int max30102_read_sample(uint32_t *ir, uint32_t *red)
{
    uint8_t buf[6]={0};
    int err = i2c_read_multi(MAX30102_ADDR, REG_FIFO_DATA, buf, 6);
    if (err) return err;

    uint32_t red24 = ((uint32_t)buf[0] << 16) | ((uint32_t)buf[1] << 8) | buf[2];
    uint32_t ir24  = ((uint32_t)buf[3] << 16) | ((uint32_t)buf[4] << 8) | buf[5];
    red24 &= 0x03FFFF; ir24 &= 0x03FFFF; /* 18-bit effective */

    if (red) *red = red24;
    if (ir)  *ir  = ir24;
    return 0;
}

/* 50 Hz sampler -> push into ring; periodically compute HR/SpO2 */
static void ppg_work_fn(struct k_work *work)
{
    ARG_UNUSED(work);

    uint32_t ir = 0, red = 0;
    if (max30102_read_sample(&ir, &red) == 0) {
        ir  = clamp_u32(ir,  0, 0x03FFFF);
        red = clamp_u32(red, 0, 0x03FFFF);
        ring_push(ir, red);

        /* Use static window buffers instead of big stack arrays */
        int n = copy_window(ppg_ir_win, ppg_red_win);
        if (n > 0) {
            uint8_t hr = 0, s2 = 0;
            bool hr_ok = false, s2_ok = false;
            if (analyze_ppg(ppg_ir_win, ppg_red_win, n,
                            &hr, &hr_ok, &s2, &s2_ok) == 0) {
                if (hr_ok) { g_hr_bpm = hr; g_hr_ok = true; }
                if (s2_ok) { g_spo2   = s2; g_s2_ok = true; }
            }
        }
    }

    k_work_schedule(&ppg_work, K_MSEC(PPG_DT_MS));
}

/* ===== Public facade ===================================================== */
int sensors_init(const struct device *i2c_dev)
{
    ARG_UNUSED(i2c_dev); /* we use DT alias-resolved g_i2c directly */

    if (!device_is_ready(g_i2c)) {
        LOG_ERR("I2C alias i2c-ppg not ready");
        return -ENODEV;
    }

    /* IMU minimal bring-up (optional) */
    (void)lsm6_init_minimal();

    /* MAX30102 PPG init */
    uint8_t part=0;
    (void)i2c_reg_read_u8(MAX30102_ADDR, REG_PART_ID, &part);
    int perr = max30102_init_config();
    ring_idx = 0; ring_filled = false;
    k_work_init_delayable(&ppg_work, ppg_work_fn);
    k_work_schedule(&ppg_work, K_MSEC(PPG_DT_MS));
    LOG_INF("MAX30102 init %s (PART=0x%02X)", (perr==0?"ok":"fail"), part);

    /* MAX30208 doesn’t need init for temp reads */
    LOG_INF("Sensors init done");
    return 0;
}

int sensors_read(uint8_t *hr_bpm,
                 uint8_t *spo2,
                 int16_t *skin_c_x100,
                 uint16_t *act_rms_x100)
{
    int rc = 0;
    static uint8_t  last_hr  = 76;
    static uint8_t  last_s2  = 98;

    /* Skin temp */
    if (skin_c_x100) {
        int16_t t=0;
        if (max30208_read_temp_x100(&t)==0) *skin_c_x100=t;
        else rc=-1;
    }

    /* Activity RMS (optional) */
    if (act_rms_x100) {
        uint16_t rms=0;
        if (lsm6_activity_rms_x100(&rms)==0) *act_rms_x100=rms;
        else rc=-1;
    }

    /* HR / SpO2 latest computed values */
    if (hr_bpm) {
        if (g_hr_ok) { last_hr = g_hr_bpm; }
        *hr_bpm = last_hr;
    }
    if (spo2) {
        if (g_s2_ok) { last_s2 = (g_spo2>100)?100:g_spo2; }
        *spo2 = last_s2;
    }

    return rc;
}

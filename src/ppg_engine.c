/* -----------------------------------------------------------------------------
 * PPG Engine (MAX30102) – implementation
 *  - Uses Zephyr sensor driver (CONFIG_MAX30102=y, CONFIG_SENSOR=y)
 *  - Samples at ~50 Hz in a delayable work item
 *  - Computes HR via peak spacing; SpO2 via ratio-of-ratios with median
 *    over recent beats; lightweight smoothing and sanity checks
 * -------------------------------------------------------------------------- */
#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <zephyr/drivers/sensor.h>
#include <zephyr/logging/log.h>
#include <math.h>
#include <string.h>
#include "ppg_engine.h"

LOG_MODULE_REGISTER(ppg, LOG_LEVEL_INF);

/* ===== Build-time feature check ========================================== */
#if !defined(CONFIG_SENSOR) || \
    !DT_HAS_COMPAT_STATUS_OKAY(maxim_max30102)
#warning "PPG engine built without DT 'maxim,max30102'; it will return -ENODEV."
#endif

/* ===== Sampler config ===================================================== */
#define PPG_SPS_HZ          50           /* target sample rate */
#define PPG_DT_MS           (1000/PPG_SPS_HZ)
#define PPG_WIN_SEC         4            /* analysis window in seconds */
#define PPG_WIN_LEN         (PPG_SPS_HZ * PPG_WIN_SEC)   /* 200 */
#define PPG_MIN_HR_BPM      30
#define PPG_MAX_HR_BPM      220
#define PPG_MIN_BEATS       2            /* need ≥2 intervals */

/* ===== Internal ring buffer ============================================== */
static uint32_t ring_ir[PPG_WIN_LEN];
static uint32_t ring_red[PPG_WIN_LEN];
static uint16_t ring_idx;
static bool     ring_filled;

/* ===== MAX30102 device handle ============================================ */
#if defined(CONFIG_SENSOR) && DT_HAS_COMPAT_STATUS_OKAY(maxim_max30102)
static const struct device *dev_ppg = DEVICE_DT_GET_ONE(maxim_max30102);
#else
/* stub build: dev_ppg unused */
#endif


/* ===== Sampler worker ===================================================== */
static struct k_work_delayable ppg_work;

/* Forward decl */
static void ppg_sample_fn(struct k_work *work);

/* ===== Small helpers ====================================================== */
static inline uint32_t clamp_u32(uint32_t v, uint32_t lo, uint32_t hi) {
    return (v < lo) ? lo : (v > hi ? hi : v);
}

static void ring_push(uint32_t ir, uint32_t red)
{
    ring_ir[ring_idx]  = ir;
    ring_red[ring_idx] = red;
    ring_idx = (ring_idx + 1) % PPG_WIN_LEN;
    if (ring_idx == 0) ring_filled = true;
}

/* Copy the most recent window into contiguous arrays for analysis */
static int copy_window(uint32_t *ir_out, uint32_t *red_out)
{
    if (!ring_filled && ring_idx < PPG_WIN_LEN/2) return -EAGAIN; /* warm-up */
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

/* ===== Core algorithm =====================================================
 * - Detrend (remove DC mean)
 * - 4-sample moving average smoothing
 * - Valley/peak detection on IR (inverted) with min distance
 * - HR from mean inter-peak distance
 * - SpO2 from ratio-of-ratios: median across valid beats
 * This is a clean-room, simplified approach suitable for demo-quality data.
 * Fine-tuning thresholds may be needed per board/LED current.
 * ======================================================================= */

/* simple ascending sort for small arrays */
static void isort_int(int *a, int n) {
    for (int i=1;i<n;i++){int t=a[i],j=i-1;while(j>=0&&a[j]>t){a[j+1]=a[j];j--;}a[j+1]=t;}
}

/* returns #peaks found; peaks[] hold indices */
static int find_peaks(const int32_t *x, int n, int min_height, int min_dist, int *peaks, int maxp)
{
    int np=0;
    for (int i=1;i<n-1;i++) {
        if (x[i] > min_height && x[i] > x[i-1] && x[i] >= x[i+1]) {
            /* enforce min distance: keep only dominant in the neighborhood */
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
    /* sort ascending */
    isort_int(peaks, np);
    return np;
}

/* robust median of small int array */
static int median_int(const int *a, int n) {
    if (n<=0) return 0;
    int tmp[8]; /* enough for our use */
    int m = (n > 8) ? 8 : n;
    for (int i=0;i<m;i++) tmp[i]=a[i];
    isort_int(tmp, m);
    return (m&1)? tmp[m/2] : (tmp[m/2-1]+tmp[m/2])/2;
}

/* Core analysis – returns 0 if both metrics computed or partially valid */
static int analyze_ppg(const uint32_t *ir_u, const uint32_t *red_u, int n,
                       uint8_t *hr_bpm, bool *hr_ok,
                       uint8_t *spo2_pct, bool *spo2_ok)
{
    if (n < PPG_SPS_HZ*2) return -EAGAIN;

    /* 1) Detrend and smooth IR (also RED for AC/DC later) */
    static int32_t ir[PPG_WIN_LEN], red[PPG_WIN_LEN];
    int64_t sum_ir=0, sum_red=0;
    for (int i=0;i<n;i++){sum_ir+=ir_u[i];sum_red+=red_u[i];}
    const int32_t mean_ir  = (int32_t)(sum_ir / n);
    const int32_t mean_red = (int32_t)(sum_red / n);

    for (int i=0;i<n;i++){ ir[i]  = (int32_t)ir_u[i]  - mean_ir;
                           red[i] = (int32_t)red_u[i] - mean_red; }
    /* 4-sample moving average (very light LPF) */
    for (int i=0;i<n-3;i++){ ir[i]  = (ir[i]+ir[i+1]+ir[i+2]+ir[i+3]) / 4;
                             red[i] = (red[i]+red[i+1]+red[i+2]+red[i+3]) / 4; }

    /* 2) Peak detection on inverted IR → valleys in original */
    static int32_t inv_ir[PPG_WIN_LEN];
    for (int i=0;i<n;i++) inv_ir[i] = -ir[i];

    /* automatic height threshold from average absolute value */
    int64_t abs_acc=0;
    for (int i=0;i<n;i++) abs_acc += (inv_ir[i]>=0 ? inv_ir[i] : -inv_ir[i]);
    int min_height = (int)(abs_acc / n);
    if (min_height < 10) min_height = 10;  /* floor */
    if (min_height > 2000) min_height = 2000;

    /* refractory = 0.3 s → samples */
    const int min_dist = (int)(PPG_SPS_HZ * 0.3f);

    int peaks[16]; /* enough for 4 s @ 50 Hz */
    int np = find_peaks(inv_ir, n, min_height, min_dist, peaks, 16);

    /* 3) HR from inter-peak distance */
    *hr_ok = false; *spo2_ok = false;
    if (np >= PPG_MIN_BEATS+1) {
        int intervals[16];
        int ni=0;
        for (int i=1;i<np;i++) {
            const int d = peaks[i] - peaks[i-1];
            if (d > 0) intervals[ni++] = d;
        }
        if (ni > 0) {
            /* mean interval → BPM */
            double mean_samp=0.0;
            for (int i=0;i<ni;i++) mean_samp += intervals[i];
            mean_samp /= ni;
            double bpm = 60.0 * (double)PPG_SPS_HZ / mean_samp;
            if (bpm >= PPG_MIN_HR_BPM && bpm <= PPG_MAX_HR_BPM) {
                *hr_bpm = (uint8_t)lround(bpm);
                *hr_ok  = true;
            }
        }
    }

    /* 4) SpO2 via ratio-of-ratios over each beat segment
          R = (ACred/DCred) / (ACir/DCir) ; map R→SpO2 with a simple fit
       We use a linearized mapping typical for demo: SpO2 ≈ 110 - 25*R
       (You can swap this with a calibrated table later.)
    */
    if (np >= PPG_MIN_BEATS+1) {
        int ratios_q7[8]; int nr=0; /* store 100*R for small ints */
        for (int k=0; k<np-1 && nr<8; k++) {
            int a = peaks[k], b = peaks[k+1];
            if (b - a < min_dist) continue;

            /* DC as max over [a,b); AC = peak-to-peak relative to linear baseline */
            int32_t ir_max = -2147483647, red_max = -2147483647;
            for (int i=a;i<b;i++){ if (ir[i]  > ir_max)  ir_max  = ir[i];
                                   if (red[i] > red_max) red_max = red[i]; }
            /* Use RMS as AC proxy for robustness */
            double ir_rms=0, red_rms=0;
            for (int i=a;i<b;i++){ ir_rms  += (double)ir[i]*ir[i];
                                   red_rms += (double)red[i]*red[i]; }
            int len = b-a;
            if (len < 4) continue;
            ir_rms  = sqrt(ir_rms/len);
            red_rms = sqrt(red_rms/len);

            /* Guard */
            if (ir_max==0 || red_max==0 || ir_rms<=1e-3 || red_rms<=1e-3) continue;

            double R = (red_rms / (double)red_max) / (ir_rms / (double)ir_max);
            int Rq = (int)lround(R * 100.0);
            if (Rq > 0 && Rq < 300) ratios_q7[nr++] = Rq;
        }

        if (nr > 0) {
            int Rmed = median_int(ratios_q7, nr);   /* median 100*R */
            double R = Rmed / 100.0;
            /* Demo mapping; refine with calibration later */
            double spo2 = 110.0 - 25.0*R;
            if (spo2 < 70.0) spo2 = 70.0;
            if (spo2 > 100.0) spo2 = 100.0;
            *spo2_pct = (uint8_t)lround(spo2);
            *spo2_ok  = true;
        }
    }

    return (*hr_ok || *spo2_ok) ? 0 : -EAGAIN;
}

/* ===== Public API ========================================================= */
int ppg_engine_init(void)
{
    ring_idx = 0; ring_filled = false;
#if defined(CONFIG_SENSOR) && defined(CONFIG_MAX30102)
    dev_ppg = DEVICE_DT_GET_ONE(maxim_max30102);
    if (!device_is_ready(dev_ppg)) {
        LOG_WRN("MAX30102 not ready");
        return -ENODEV;
    }
    /* Start sampler */
    k_work_init_delayable(&ppg_work, ppg_sample_fn);
    k_work_schedule(&ppg_work, K_MSEC(PPG_DT_MS));
    LOG_INF("PPG engine started (MAX30102, %d Hz)", PPG_SPS_HZ);
    return 0;
#else
    LOG_WRN("PPG engine compiled without MAX30102 support");
    return -ENODEV;
#endif
}

void ppg_engine_stop(void)
{
    k_work_cancel_delayable(&ppg_work);
}

int ppg_engine_latest(uint8_t *hr_bpm, bool *hr_ok,
                      uint8_t *spo2_pct, bool *spo2_ok)
{
    uint32_t ir[PPG_WIN_LEN], red[PPG_WIN_LEN];
    int n = copy_window(ir, red);
    if (n < 0) return n;
    return analyze_ppg(ir, red, n, hr_bpm, hr_ok, spo2_pct, spo2_ok);
}

/* ===== Sampling worker: fetch a sample and push to ring =================== */
static void ppg_sample_fn(struct k_work *work)
{
    ARG_UNUSED(work);

#if defined(CONFIG_SENSOR) && defined(CONFIG_MAX30102)
    struct sensor_value ir_sv = {0}, red_sv = {0};

    /* Fetch latest sample; if sensor is set to FIFO mode with >1 sample,
       the driver typically returns the newest processed values. */
    if (sensor_sample_fetch(dev_ppg) == 0) {
        /* Prefer RAW channels if driver exposes them; fall back to IR/RED. */
#ifdef SENSOR_CHAN_IR
        (void)sensor_channel_get(dev_ppg, SENSOR_CHAN_IR, &ir_sv);
#endif
#ifdef SENSOR_CHAN_RED
        (void)sensor_channel_get(dev_ppg, SENSOR_CHAN_RED, &red_sv);
#endif
        /* Convert to integer counts; if driver already reports counts in .val1,
           keep it; otherwise scale from micro-units. */
        uint32_t ir  = (ir_sv.val1 > 0) ? (uint32_t)ir_sv.val1
                                        : (uint32_t)(ir_sv.val2 / 1000); /* best-effort */
        uint32_t red = (red_sv.val1 > 0) ? (uint32_t)red_sv.val1
                                         : (uint32_t)(red_sv.val2 / 1000);

        ir  = clamp_u32(ir,  0, 0x03FFFF);   /* MAX30102 is 18-bit effective */
        red = clamp_u32(red, 0, 0x03FFFF);
        ring_push(ir, red);
    }
    /* reschedule */
    k_work_schedule(&ppg_work, K_MSEC(PPG_DT_MS));
#endif
}

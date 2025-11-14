#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(drv2605, LOG_LEVEL_INF);

#include "drv2605.h"
#include <zephyr/sys/util.h>   /* CLAMP(), MIN() */
#include <zephyr/kernel.h>
#include <zephyr/drivers/i2c.h>
#include <zephyr/drivers/gpio.h>

/* ===== Register map (subset) ===== */
#define DRV2605_REG_STATUS          0x00
#define DRV2605_REG_MODE            0x01
#define DRV2605_REG_RTP_INPUT       0x02
#define DRV2605_REG_LIBRARY         0x03
#define DRV2605_REG_WAVESEQ0        0x04 /* ..0x0B */
#define DRV2605_REG_GO              0x0C
#define DRV2605_REG_ODT             0x0D
#define DRV2605_REG_SPT             0x0E
#define DRV2605_REG_SNT             0x0F
#define DRV2605_REG_BRT             0x10
#define DRV2605_REG_RATED_V         0x16
#define DRV2605_REG_OD_CLAMP        0x17
#define DRV2605_REG_A_CAL_COMP      0x18
#define DRV2605_REG_A_CAL_BEMF      0x19
#define DRV2605_REG_FEEDBACK        0x1A
#define DRV2605_REG_CONTROL1        0x1B
#define DRV2605_REG_CONTROL2        0x1C
#define DRV2605_REG_CONTROL3        0x1D
#define DRV2605_REG_CONTROL4        0x1E
#define DRV2605_REG_VBAT            0x21
#define DRV2605_REG_LRA_PERIOD      0x22

/* MODE[2:0] values */
#define DRV2605_MODE_INT_TRIG       0x00
#define DRV2605_MODE_EXT_EDGE       0x01
#define DRV2605_MODE_EXT_LEVEL      0x02
#define DRV2605_MODE_PWM_ANALOG     0x03
#define DRV2605_MODE_AUDIO          0x04
#define DRV2605_MODE_RTP            0x05
#define DRV2605_MODE_DIAG           0x06
#define DRV2605_MODE_AUTOCAL        0x07

/* Bit helpers */
#define BIT_DEV_RESET               0x80
#define BIT_STANDBY                 0x40
#define BIT_GO                      0x01

/* DT node */
#define DRV2605_NODE DT_NODELABEL(haptic)

static struct {
    struct i2c_dt_spec i2c;
    struct gpio_dt_spec en;
    struct drv2605_cfg cfg;
    bool ready;
} g;

/* ===== I2C helpers ===== */
static inline int reg_w(uint8_t reg, uint8_t val)  { return i2c_reg_write_byte_dt(&g.i2c, reg, val); }
static inline int reg_r(uint8_t reg, uint8_t *val) { return i2c_reg_read_byte_dt (&g.i2c, reg, val); }

static int await_go_clear(k_timeout_t to)
{
    int64_t end = k_uptime_get() + k_ticks_to_ms_ceil32(to.ticks);
    uint8_t v;
    do {
        int err = reg_r(DRV2605_REG_GO, &v);
        if (err) return err;
        if ((v & BIT_GO) == 0) return 0;
        k_msleep(5);
    } while (k_uptime_get() < end);
    return -ETIMEDOUT;
}

static int probe_status_device_id(uint8_t *dev_id)
{
    uint8_t st = 0;
    int err = reg_r(DRV2605_REG_STATUS, &st);
    if (err) return err;
    if (dev_id) *dev_id = (st >> 5) & 0x07; /* 3 = DRV2605, 7 = DRV2605L */
    return 0;
}

/* ===== Encoding helpers (per datasheet’s ~21 mV/LSB scaling) =====
   These are conservative mappings that work broadly. */
static uint8_t encode_rated_mv_erm(uint16_t mv) { return (uint8_t)CLAMP(mv / 21, 1, 255); }
static uint8_t encode_rated_mv_lra(uint16_t mv) { return (uint8_t)CLAMP(mv / 21, 1, 255); }
static uint8_t encode_od_mv(uint16_t mv)        { return (uint8_t)CLAMP(mv / 22, 1, 255); }

/* LRA DRIVE_TIME code from frequency:
 * DRIVE_TIME (ms) ≈ 0.5 * period; code = (DriveTime_ms - 0.5)/0.1
 * Using integer math: code ≈ round(5000/hz) - 5, clamp [0..31].  */
static uint8_t lra_drive_time_code_for_hz(unsigned hz)
{
    if (hz < 125) hz = 125;
    if (hz > 300) hz = 300;
    int code = (int)((5000u + hz/2) / hz) - 5;
    if (code < 0) code = 0;
    if (code > 31) code = 31;
    return (uint8_t)code;
}

int drv2605_enable(bool on)
{
    if (!device_is_ready(g.en.port)) return -ENODEV;
    int err = gpio_pin_configure_dt(&g.en, GPIO_OUTPUT_INACTIVE);
    if (err) return err;
    return gpio_pin_set_dt(&g.en, on ? 1 : 0);
}

/* ===== Base init (power up, leave standby, basic config) ===== */
static int base_init(void)
{
    if (!device_is_ready(g.i2c.bus)) return -ENODEV;
    if (!device_is_ready(g.en.port)) return -ENODEV;

    /* EN high to wake device. */
    int err = drv2605_enable(true);
    if (err) return err;

    /* Margin for power/REG settle (datasheet requires >=250 µs). */
    k_msleep(3);

    /* Probe with a few retries—if needed, toggle EN to re-wake. */
    for (int attempt = 0; attempt < 3; ++attempt) {
        uint8_t id = 0;
        err = probe_status_device_id(&id);
        if (!err && (id == 3 || id == 7)) break;
        gpio_pin_set_dt(&g.en, 0);
        k_msleep(2);
        gpio_pin_set_dt(&g.en, 1);
        k_msleep(3);
        err = -ENODEV;
    }
    if (err) return err;

    /* Leave standby, MODE=0 (internal trigger). */
    err = reg_w(DRV2605_REG_MODE, DRV2605_MODE_INT_TRIG);
    if (err) return err;

    /* FEEDBACK (0x1A): set actuator + defaults (brake factor, loop gain). */
    uint8_t fb = 0x00;
    if (g.cfg.actuator == DRV2605_ACTUATOR_LRA) fb |= 0x80; /* N_ERM_LRA=1 -> LRA */
    fb |= (3 << 4); /* FB_BRAKE_FACTOR=4x */
    fb |= (1 << 2); /* LOOP_GAIN=Medium */
    fb |= (2 << 0); /* BEMF_GAIN default */
    err = reg_w(DRV2605_REG_FEEDBACK, fb);
    if (err) return err;

    /* CONTROL1 (0x1B): STARTUP_BOOST=1, set DRIVE_TIME. */
    uint8_t drive_time_code = 0;
    if (g.cfg.actuator == DRV2605_ACTUATOR_LRA) {
        if (g.cfg.drive_time_tenth_ms) {
            drive_time_code = CLAMP(g.cfg.drive_time_tenth_ms, 0, 31);
        } else {
            unsigned hz = g.cfg.lra_hz ? g.cfg.lra_hz : 175;
            drive_time_code = lra_drive_time_code_for_hz(hz);
        }
    } else {
        /* ERM: use provided drive_time_tenth_ms if any, else default 0x13. */
        drive_time_code = g.cfg.drive_time_tenth_ms ? CLAMP(g.cfg.drive_time_tenth_ms, 0, 31) : 0x13;
    }
    uint8_t c1 = (1u << 7) | (drive_time_code & 0x1F);
    err = reg_w(DRV2605_REG_CONTROL1, c1);
    if (err) return err;

    /* CONTROL2 (0x1C): default good values. */
    err = reg_w(DRV2605_REG_CONTROL2, 0xF5);
    if (err) return err;

    /* CONTROL3 (0x1D): Set OPEN-LOOP mode for LRA/ERM and defaults */
    uint8_t c3 = 0xA0; /* Base: NG_THRESH=4% and default flags */
    if (g.cfg.actuator == DRV2605_ACTUATOR_LRA) {
        c3 |= 0x01;        /* LRA_OPEN_LOOP = 1 (bit0) */
        c3 &= ~(1u << 5);  /* ERM_OPEN_LOOP = 0 (bit5) */
        LOG_INF("DRV2605: Configured for OPEN-LOOP LRA mode (skipping auto-calibration)");
    } else {
        c3 |= (1u << 5);   /* ERM_OPEN_LOOP = 1 (bit5) */
        c3 &= ~0x01;       /* LRA_OPEN_LOOP = 0 (bit0) */
        LOG_INF("DRV2605: Configured for OPEN-LOOP ERM mode (skipping auto-calibration)");
    }
    err = reg_w(DRV2605_REG_CONTROL3, c3);
    if (err) return err;

    /* Rated voltage & overdrive clamp (used even in open-loop for drive strength) */
    uint8_t rated = (g.cfg.actuator == DRV2605_ACTUATOR_LRA)
        ? encode_rated_mv_lra(g.cfg.rated_mv ? g.cfg.rated_mv : 2000)
        : encode_rated_mv_erm(g.cfg.rated_mv ? g.cfg.rated_mv : 2000);
    uint8_t clamp = encode_od_mv(g.cfg.od_clamp_mv ? g.cfg.od_clamp_mv
                                                   : (uint16_t)((g.cfg.rated_mv ? g.cfg.rated_mv : 2000) * 13 / 10));
    err = reg_w(DRV2605_REG_RATED_V, rated);
    if (err) return err;
    err = reg_w(DRV2605_REG_OD_CLAMP, clamp);
    if (err) return err;

    g.ready = true;
    return 0;
}

/* ===== Auto-cal helpers ===== */
/* Run one auto-cal pass and check DIAG_RESULT (STATUS[3]). */
/* Auto-calibration functions removed - using open-loop mode instead */

int drv2605_auto_calibrate(void)
{
    if (!g.ready) return -EAGAIN;

    /* Use the longest auto-cal time window (CONTROL4[5:4] = 3). */
    uint8_t c4 = 0;
    (void)reg_r(DRV2605_REG_CONTROL4, &c4);
    c4 &= ~(0x3 << 4);
    c4 |=  (0x3 << 4);
    int err = reg_w(DRV2605_REG_CONTROL4, c4);
    if (err) return err;

    /* Read the currently programmed DRIVE_TIME from CONTROL1[4:0].
     * This avoids depending on any local variable from base_init(). */
    uint8_t c1_val = 0;
    err = reg_r(DRV2605_REG_CONTROL1, &c1_val);
    if (err) return err;
    uint8_t base_dt_code = (uint8_t)(c1_val & 0x1F);

    /* Build a small sweep around the current DRIVE_TIME to help convergence. */
    const uint8_t dt_try_code[] = {
        base_dt_code,
        (uint8_t)CLAMP((int)base_dt_code - 4, 0, 31),
        (uint8_t)CLAMP((int)base_dt_code + 4, 0, 31),
        (uint8_t)CLAMP((int)base_dt_code + 8, 0, 31),
    };

    /* Try a couple of rated voltages; clamp ≈ 130% of rated. (Write 0x16/0x17 *before* auto-cal.) */
    const uint16_t base_rated_mv = g.cfg.rated_mv ? g.cfg.rated_mv : 1800;
    const uint16_t rated_try_mv[] = { base_rated_mv, 1500, 2000 };

    for (size_t i = 0; i < ARRAY_SIZE(rated_try_mv); ++i) {
        uint16_t r_mv = rated_try_mv[i];
        uint16_t c_mv = (uint16_t)(r_mv * 13 / 10);

        /* Program RATED_V (0x16) and OD_CLAMP (0x17). */
        uint8_t rated = (g.cfg.actuator == DRV2605_ACTUATOR_LRA)
            ? (uint8_t)CLAMP(r_mv / 21, 1, 255)     /* ~21 mV/LSB */
            : (uint8_t)CLAMP(r_mv / 21, 1, 255);
        uint8_t clamp = (uint8_t)CLAMP(c_mv / 22, 1, 255); /* ~22 mV/LSB */

        err = reg_w(DRV2605_REG_RATED_V, rated);
        if (err) return err;
        err = reg_w(DRV2605_REG_OD_CLAMP, clamp);
        if (err) return err;

        for (size_t j = 0; j < ARRAY_SIZE(dt_try_code); ++j) {
            /* Program CONTROL1: keep STARTUP_BOOST=1, update DRIVE_TIME. */
            uint8_t c1_new = (1u << 7) | (dt_try_code[j] & 0x1F);
            err = reg_w(DRV2605_REG_CONTROL1, c1_new);
            if (err) return err;

            /* MODE=AUTOCAL, GO=1, wait for GO to clear, then check STATUS.DIAG_RESULT. */
            err = reg_w(DRV2605_REG_MODE, DRV2605_MODE_AUTOCAL);
            if (err) return err;
            err = reg_w(DRV2605_REG_GO, BIT_GO);
            if (err) return err;
            err = await_go_clear(K_SECONDS(2));
            if (err) return err;

            uint8_t st = 0;
            err = reg_r(DRV2605_REG_STATUS, &st);
            if (err) return err;

            if ((st & 0x08) == 0) {
                uint8_t comp=0, bemf=0;
                (void)reg_r(DRV2605_REG_A_CAL_COMP, &comp);
                (void)reg_r(DRV2605_REG_A_CAL_BEMF, &bemf);
                LOG_INF("DRV2605 auto-cal OK: rated=%umV dt=%u (COMP=0x%02x BEMF=0x%02x ST=0x%02x)",
                        r_mv, dt_try_code[j], comp, bemf, st);
                return 0;
            }

            LOG_WRN("Auto-cal try failed (STATUS=0x%02x): rated=%u dt=%u; retrying…", st, r_mv, dt_try_code[j]);
        }
    }

    uint8_t st=0; (void)reg_r(DRV2605_REG_STATUS, &st);
    LOG_ERR("DRV2605 auto-cal failed (STATUS=0x%02x) — switching to OPEN-LOOP fallback", st);

    /* Force open-loop so you can still test effects/RTP. */
    uint8_t c3 = 0;
    if (!reg_r(DRV2605_REG_CONTROL3, &c3)) {
        if (g.cfg.actuator == DRV2605_ACTUATOR_LRA) {
            c3 |= 0x01;        /* LRA_OPEN_LOOP = 1 */
            c3 &= ~(1u << 5);  /* ERM_OPEN_LOOP = 0 */
        } else {
            c3 |= (1u << 5);   /* ERM_OPEN_LOOP = 1 */
            c3 &= ~0x01;       /* LRA_OPEN_LOOP = 0 */
        }
        (void)reg_w(DRV2605_REG_CONTROL3, c3);
    }
    return -EIO;
}

/* ===== Playback APIs ===== */
int drv2605_select_library(uint8_t lib)
{
    /* 1..5 = ERM libs; 6 = LRA library. */
    if (lib < 1 || lib > 6) return -EINVAL;
    return reg_w(DRV2605_REG_LIBRARY, lib);
}

int drv2605_play_effect(uint8_t effect_id)
{
    if (effect_id == 0 || effect_id > 123) return -EINVAL;
    int err = reg_w(DRV2605_REG_WAVESEQ0 + 0, effect_id);
    if (err) return err;
    for (int i=1; i<8; ++i) {
        err = reg_w(DRV2605_REG_WAVESEQ0 + i, 0x00);
        if (err) return err;
    }
    err = reg_w(DRV2605_REG_MODE, DRV2605_MODE_INT_TRIG);
    if (err) return err;
    return reg_w(DRV2605_REG_GO, BIT_GO);
}

int drv2605_stop(void)
{
    /* Clear to internal trigger idle (STOP). */
    return reg_w(DRV2605_REG_MODE, DRV2605_MODE_INT_TRIG);
}

int drv2605_rtp_begin(void)
{
    uint8_t c3;
    int err = reg_r(DRV2605_REG_CONTROL3, &c3);
    if (err) return err;
    c3 |= (1u << 3); /* DATA_FORMAT_RTP=1 -> unsigned */
    err = reg_w(DRV2605_REG_CONTROL3, c3);
    if (err) return err;

    err = reg_w(DRV2605_REG_MODE, DRV2605_MODE_RTP);
    if (err) return err;
    return reg_w(DRV2605_REG_RTP_INPUT, 0);
}

int drv2605_rtp_write(uint8_t amp)
{
    return reg_w(DRV2605_REG_RTP_INPUT, amp);
}

int drv2605_rtp_end(void)
{
    return drv2605_stop();
}

/* ===== Public initialization ===== */
int drv2605_init_explicit(const struct i2c_dt_spec *i2c, const struct gpio_dt_spec *en_gpio,
                          const struct drv2605_cfg *cfg)
{
    g.i2c = *i2c;
    g.en  = *en_gpio;
    g.cfg = *cfg;

    int err = base_init();
    if (err) return err;

    /* Skip auto-calibration - using open-loop mode configured in base_init() */
    LOG_INF("DRV2605: Initialized successfully in OPEN-LOOP mode");

    /* Select library (LRA=6, ERM=4) */
    err = drv2605_select_library((g.cfg.actuator == DRV2605_ACTUATOR_LRA) ? 6 : 4);
    if (err) return err;

    return 0;
}

int drv2605_init_from_dt(const struct device *unused)
{
    ARG_UNUSED(unused);
#if !DT_NODE_EXISTS(DRV2605_NODE)
# error "DRV2605 node 'haptic' not found in device tree"
#endif
    static const struct i2c_dt_spec i2c = I2C_DT_SPEC_GET(DRV2605_NODE);
    static const struct gpio_dt_spec en  = GPIO_DT_SPEC_GET(DRV2605_NODE, en_gpios);

    struct drv2605_cfg cfg = {
#ifdef DT_PROP(DRV2605_NODE, ti_lra)
        .actuator = DRV2605_ACTUATOR_LRA,
#else
        .actuator = DRV2605_ACTUATOR_ERM,
#endif
        .rated_mv = DT_PROP_OR(DRV2605_NODE, ti_rated_mv, 2000),
        .od_clamp_mv = DT_PROP_OR(DRV2605_NODE, ti_od_clamp_mv, 2600),
        .drive_time_tenth_ms = DT_PROP_OR(DRV2605_NODE, ti_drive_time_tenth_ms, 0),
        .auto_cal_time_sel   = DT_PROP_OR(DRV2605_NODE, ti_auto_cal_time, 3),
        .lra_hz = DT_PROP_OR(DRV2605_NODE, ti_lra_hz, 175),
    };

    return drv2605_init_explicit(&i2c, &en, &cfg);
}

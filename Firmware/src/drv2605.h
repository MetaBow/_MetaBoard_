#pragma once
#include <zephyr/device.h>
#include <zephyr/kernel.h>
#include <zephyr/drivers/i2c.h>
#include <zephyr/drivers/gpio.h>

#ifdef __cplusplus
extern "C" {
#endif

enum drv2605_actuator {
    DRV2605_ACTUATOR_LRA = 0,
    DRV2605_ACTUATOR_ERM = 1,
};

struct drv2605_cfg {
    enum drv2605_actuator actuator; /* DRV2605_ACTUATOR_LRA or _ERM */
    uint16_t rated_mv;              /* mV, steady-state */
    uint16_t od_clamp_mv;           /* mV, overdrive clamp */
    uint8_t  drive_time_tenth_ms;   /* 0..31; if 0 for LRA we compute from lra_hz */
    uint8_t  auto_cal_time_sel;     /* 0..3; we override to longest in driver */
    uint16_t lra_hz;                /* NEW: typical 160–200 Hz; 0=use default 175 */
};


int drv2605_init_from_dt(const struct device *unused);
int drv2605_init_explicit(const struct i2c_dt_spec *i2c, const struct gpio_dt_spec *en_gpio,
                          const struct drv2605_cfg *cfg);

/* One-shot playback using ROM libraries */
int drv2605_select_library(uint8_t lib);            /* 1..5 ERM libs, 6 = LRA */
int drv2605_play_effect(uint8_t effect_id);         /* 1..123 (exact list in datasheet) */
int drv2605_stop(void);

/* RTP (real-time playback) drive: 0..255 unsigned in closed-loop */
int drv2605_rtp_begin(void);
int drv2605_rtp_write(uint8_t amp);
int drv2605_rtp_end(void);

/* Power gating */
int drv2605_enable(bool on);

/* Diagnostics */
int drv2605_auto_calibrate(void);                   /* runs auto-cal (blocking) */
int drv2605_diagnostics(void);                      /* returns 0 if pass, -EIO if fault */

#ifdef __cplusplus
}
#endif

#include <string.h>
#include <stdlib.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(haptic_cmd, LOG_LEVEL_INF);

#include "drv2605.h"

static int to_uint(const char *s, unsigned *out)
{
    char *end = NULL;
    unsigned long v = strtoul(s, &end, 10);
    if (end == s || *end != '\0') return -EINVAL;
    *out = (unsigned)v;
    return 0;
}

bool haptic_cmd_handle(const char *cmd_in)
{
    if (!cmd_in) return false;

    /* Copy, trim, and convert to uppercase */
    char buf[64];
    size_t n = MIN(strlen(cmd_in), sizeof(buf)-1);
    memcpy(buf, cmd_in, n); buf[n] = 0;
    
    /* Remove trailing whitespace, newlines, carriage returns */
    for (char *p = buf; *p; ++p) {
        if (*p == '\r' || *p == '\n') *p = 0;
        else if (*p >= 'a' && *p <= 'z') *p = *p - 32; /* Convert to uppercase */
    }
    
    /* Trim trailing spaces */
    for (int i = strlen(buf) - 1; i >= 0 && buf[i] == ' '; i--) buf[i] = 0;
    
    /* Log received command for debugging */
    LOG_DBG("HAP cmd: '%s'", buf);

    char *tok = strtok(buf, " ");
    if (!tok) return false;

    if (!strcmp(tok, "BUZZ")) {
        char *ms_s = strtok(NULL, " ");
        char *amp_s = strtok(NULL, " ");
        if (!ms_s) return false;
        unsigned ms=0, amp=180;
        if (to_uint(ms_s, &ms)) return false;
        if (amp_s && to_uint(amp_s, &amp)) return false;
        if (!drv2605_rtp_begin()) {
            drv2605_rtp_write((uint8_t)CLAMP(amp, 0, 255));
            k_msleep(ms);
            drv2605_rtp_end();
            LOG_INF("HAP: buzz %ums @%u", ms, amp);
            return true;
        }
        return false;
    }

    /* All other commands intentionally disabled */
    LOG_WRN("HAP: command disabled '%s'", tok);
    return false;
}

/*
 * protocol.c - see protocol.h.
 *
 * Commands (plain text lines):
 *   id | status | selftest | ping | dump | start | stop | light on|off
 *   set freq <Hz> | set gain <dB> | set burst <N> | set heartbeat <s>
 *   set hpf 12|96|384|0 | set ch2 0|1 | bootsel | reset
 * Replies are single JSON lines (type: id, status, selftest, ack, regs, error,
 * log) and unsolicited data / burst messages while streaming.
 */
#include "protocol.h"
#include "app.h"
#include "i2s_rx.h"
#include "lockin.h"
#include "modulator.h"
#include "selftest.h"
#include "usb_io.h"

#include <ctype.h>
#include <math.h>
#include <stdarg.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "hardware/clocks.h"
#include "hardware/watchdog.h"
#include "pico/bootrom.h"
#include "pico/time.h"
#include "pico/unique_id.h"

#define MSG_MAX 16384

static char msg[MSG_MAX];
static size_t msg_len;

static const char *b(bool v) { return v ? "true" : "false"; }

static void json_escape(char *dst, size_t n, const char *src) {
    size_t o = 0;
    for (; *src && o + 2 < n; src++) {
        unsigned char c = (unsigned char)*src;
        if (c == '"' || c == '\\') {
            if (o + 3 >= n) break;
            dst[o++] = '\\';
            dst[o++] = (char)c;
        } else if (c < 0x20) {
            dst[o++] = ' ';
        } else {
            dst[o++] = (char)c;
        }
    }
    dst[o] = 0;
}

static void send_msg(usb_msg_kind_t kind) {
    usb_send(msg, msg_len, kind);
}

static void msg_printf(const char *fmt, ...) {
    va_list ap;
    va_start(ap, fmt);
    int n = vsnprintf(msg + msg_len, MSG_MAX - msg_len, fmt, ap);
    va_end(ap);
    if (n > 0) msg_len += (size_t)n < MSG_MAX - msg_len ? (size_t)n : MSG_MAX - msg_len - 1;
}

/* ---- outgoing messages ------------------------------------------------------- */

void proto_log(const char *level, const char *fmt, ...) {
    char text[200], esc[260];
    va_list ap;
    va_start(ap, fmt);
    vsnprintf(text, sizeof text, fmt, ap);
    va_end(ap);
    json_escape(esc, sizeof esc, text);
    msg_len = 0;
    msg_printf("{\"type\":\"log\",\"level\":\"%s\",\"msg\":\"%s\"}\n", level, esc);
    send_msg(USB_MSG_CTRL);
}

void proto_error(const char *fmt, ...) {
    char text[200], esc[260];
    va_list ap;
    va_start(ap, fmt);
    vsnprintf(text, sizeof text, fmt, ap);
    va_end(ap);
    json_escape(esc, sizeof esc, text);
    msg_len = 0;
    msg_printf("{\"type\":\"error\",\"msg\":\"%s\"}\n", esc);
    send_msg(USB_MSG_CTRL);
}

void proto_send_id(void) {
    char uid[2 * PICO_UNIQUE_BOARD_ID_SIZE_BYTES + 1];
    pico_get_unique_board_id_string(uid, sizeof uid);
    msg_len = 0;
    msg_printf("{\"type\":\"id\",\"fw\":\"%s\",\"version\":\"%s\",\"proto\":%d,\"fs\":%d,\"block\":%d,"
               "\"bits\":16,\"adc_bits\":24,\"adc\":%s,\"driver\":\"vccs\",\"mod_pin\":%d,"
               "\"bclk_per_frame\":%d,\"fmin\":%.1f,\"fmax\":%.1f,\"sysclk_hz\":%lu,\"hpf_hz\":%d,"
               "\"adc_latency_samples\":%.1f,\"mod_edge_frames\":%.4f,\"table_len\":%d,"
               "\"uid\":\"%s\",\"reset_reason\":\"%s\"}\n",
               FW_NAME, FW_VERSION, PROTO_VERSION, FS_HZ, BLOCK, b(app.adc_ok), PIN_MOD,
               BCLK_PER_FRAME, F_MIN_HZ, F_MAX_HZ, (unsigned long)clock_get_hz(clk_sys),
               tlv320_hpf_hz(app.hpf_sel), ADC_LATENCY_SAMPLES, MOD_EDGE_FRAMES, TABLE_LEN,
               uid, app.wd_reboot ? "watchdog" : "power/reset");
    send_msg(USB_MSG_CTRL);
}

void proto_send_status(void) {
    const tlv320_status_t *s = &app.adc_status;
    char ratio[12];
    uint32_t w_entry, w_armed, arm_us;
    modulator_arm_debug(&w_entry, &w_armed, &arm_us);
    if (s->bclk_ratio > 0) snprintf(ratio, sizeof ratio, "%d", s->bclk_ratio);
    else snprintf(ratio, sizeof ratio, "\"invalid\"");
    msg_len = 0;
    msg_printf("{\"type\":\"status\",\"adc\":{\"mode\":\"%s\",\"ch1_on\":%s,\"ch2_on\":%s,\"fs\":\"%s\","
               "\"bclk_ratio\":%s,\"clock_ok\":%s,\"int_latch\":%u},\"adc_ok\":%s,\"freq\":%.4f,"
               "\"p_bclk\":%lu,\"gain_db\":%.1f,\"light\":%s,\"streaming\":%s,\"burst\":%lu,"
               "\"heartbeat_s\":%lu,\"seq\":%lu,\"overruns\":%lu,\"blocks_skipped\":%lu,"
               "\"i2c_errors\":%lu,\"tx_drop_burst\":%lu,\"tx_drop_data\":%lu,\"mod_sync\":\"%s\","
               "\"mod_seq\":%lu,\"block_period_us\":%lu,\"uptime_s\":%lu,\"hpf_hz\":%d,\"ch2\":%s,"
               "\"sysclk_hz\":%lu,\"mod_w_entry\":%lu,\"mod_w_armed\":%lu,\"mod_arm_us\":%lu}\n",
               s->mode ? s->mode : "?", b(s->ch1_on), b(s->ch2_on), s->fs ? s->fs : "?", ratio,
               b(s->clock_ok), s->int_latch, b(app.adc_ok), app.freq, (unsigned long)app.p_bclk,
               app.gain_db, b(app.light), b(app.streaming), (unsigned long)app.burst_every,
               (unsigned long)app.heartbeat_s, (unsigned long)app.seq, (unsigned long)i2s_overruns(),
               (unsigned long)app.blocks_skipped, (unsigned long)app.adc.i2c_errors,
               (unsigned long)usb_drop_burst(), (unsigned long)usb_drop_data(), modulator_sync_str(),
               (unsigned long)app.mod_seq, (unsigned long)i2s_block_period_us(),
               (unsigned long)((time_us_64() - app.t0_us) / 1000000), tlv320_hpf_hz(app.hpf_sel),
               b(app.ch2), (unsigned long)clock_get_hz(clk_sys), (unsigned long)w_entry,
               (unsigned long)w_armed, (unsigned long)arm_us);
    send_msg(USB_MSG_CTRL);
}

void proto_send_data(uint32_t seq, uint64_t t_us) {
    const lockin_result_t *r = &app.last;
    msg_len = 0;
    msg_printf("{\"type\":\"data\",\"seq\":%lu,\"t\":%lu,\"i\":%.1f,\"q\":%.1f,\"amp\":%.7g,"
               "\"amp_db\":%.2f,\"ph\":%.3f,\"rms\":%.7g,\"rms_db\":%.2f,\"peak\":%ld,\"ovl\":%s,"
               "\"clk\":%s,\"freq\":%.4f,\"light\":%s,\"gain\":%.1f,\"dc\":%.7g,\"mod_seq\":%lu,"
               "\"t_us\":%llu}\n",
               (unsigned long)seq, (unsigned long)((t_us - app.t0_us) / 1000), r->i, r->q, r->amp,
               dbfs(r->amp), r->ph_deg, r->rms, dbfs(r->rms), (long)r->peak16, b(r->ovl),
               b(app.adc_status.clock_ok), app.freq, b(app.light), app.gain_db, r->dc,
               (unsigned long)app.mod_seq, (unsigned long long)t_us);
    send_msg(USB_MSG_DATA);
}

static const char b64chars[] = "ABCDEFGHIJKLMNOPQRSTUVWXYZabcdefghijklmnopqrstuvwxyz0123456789+/";

/* Left slot of every frame as int16 (top 16 bits of the 24-bit sample), base64. */
void proto_send_burst(uint32_t seq, const int32_t *frames) {
    msg_len = 0;
    msg_printf("{\"type\":\"burst\",\"seq\":%lu,\"n\":%d,\"fs\":%d,\"p_bclk\":%lu,\"ph0\":%.6f,\"b64\":\"",
               (unsigned long)seq, BLOCK, FS_HZ, (unsigned long)app.li.p, app.li.ph0);
    char *o = msg + msg_len;
    uint8_t bytes[6];
    for (uint32_t n = 0; n < BLOCK; n += 3) {          /* 3 samples = 6 bytes = 8 chars */
        for (int k = 0; k < 3; k++) {
            int16_t v = (int16_t)(frames[2 * (n + k)] >> 16);
            bytes[2 * k] = (uint8_t)(v & 0xFF);
            bytes[2 * k + 1] = (uint8_t)((v >> 8) & 0xFF);
        }
        for (int k = 0; k < 6; k += 3) {
            uint32_t t = ((uint32_t)bytes[k] << 16) | ((uint32_t)bytes[k + 1] << 8) | bytes[k + 2];
            *o++ = b64chars[(t >> 18) & 63];
            *o++ = b64chars[(t >> 12) & 63];
            *o++ = b64chars[(t >> 6) & 63];
            *o++ = b64chars[t & 63];
        }
    }
    msg_len = (size_t)(o - msg);
    msg_printf("\"}\n");
    send_msg(USB_MSG_BURST);
}

/* ---- self-test list --------------------------------------------------------- */

static bool st_first;

void proto_selftest_begin(void) {
    msg_len = 0;
    msg_printf("{\"type\":\"selftest\",\"results\":[");
    st_first = true;
}

void proto_selftest_add(const char *name, const char *status, const char *detail) {
    char esc[300];
    json_escape(esc, sizeof esc, detail ? detail : "");
    msg_printf("%s{\"name\":\"%s\",\"status\":\"%s\",\"detail\":\"%s\"}", st_first ? "" : ",", name, status, esc);
    st_first = false;
}

void proto_selftest_end(void) {
    msg_printf("]}\n");
    send_msg(USB_MSG_CTRL);
}

/* ---- commands --------------------------------------------------------------- */

static void ack_bool(const char *key, bool v) {
    msg_len = 0;
    msg_printf("{\"type\":\"ack\",\"key\":\"%s\",\"value\":%s}\n", key, b(v));
    send_msg(USB_MSG_CTRL);
}

static void ack_uint(const char *key, uint32_t v) {
    msg_len = 0;
    msg_printf("{\"type\":\"ack\",\"key\":\"%s\",\"value\":%lu}\n", key, (unsigned long)v);
    send_msg(USB_MSG_CTRL);
}

static bool streq_ci(const char *a, const char *lower) {
    for (; *a && *lower; a++, lower++)
        if (tolower((unsigned char)*a) != *lower) return false;
    return *a == 0 && *lower == 0;
}

static void cmd_set(const char *key, const char *val) {
    if (streq_ci(key, "freq")) {
        double f = strtod(val, NULL);
        if (!(f >= F_MIN_HZ && f <= F_MAX_HZ)) {
            proto_error("freq %.0f..%.0f Hz", F_MIN_HZ, F_MAX_HZ);
            return;
        }
        app_set_period((uint32_t)lround((double)BCLK_HZ / f));
        msg_len = 0;
        msg_printf("{\"type\":\"ack\",\"key\":\"freq\",\"value\":%.4f,\"p_bclk\":%lu}\n", app.freq,
                   (unsigned long)app.p_bclk);
        send_msg(USB_MSG_CTRL);
    } else if (streq_ci(key, "gain")) {
        double g = strtod(val, NULL);
        int code = (int)lround(g * 2.0);
        if (!app.adc_ok) { proto_error("gain: no ADC"); return; }
        if (code < 0 || code > 84) { proto_error("gain 0..42 dB"); return; }
        app.gain_db = code / 2.0f;
        if (!tlv320_set_gain(&app.adc, 1, app.gain_db)) { proto_error("gain: I2C write failed"); return; }
        if (app.ch2) tlv320_set_gain(&app.adc, 2, app.gain_db);
        msg_len = 0;
        msg_printf("{\"type\":\"ack\",\"key\":\"gain\",\"value\":%.1f}\n", app.gain_db);
        send_msg(USB_MSG_CTRL);
    } else if (streq_ci(key, "burst")) {
        long n = strtol(val, NULL, 10);
        app.burst_every = n < 0 ? 0 : (uint32_t)n;
        ack_uint("burst", app.burst_every);
    } else if (streq_ci(key, "heartbeat")) {
        long n = strtol(val, NULL, 10);
        app.heartbeat_s = n < 0 ? 0 : (uint32_t)n;
        ack_uint("heartbeat", app.heartbeat_s);
    } else if (streq_ci(key, "hpf")) {
        long hz = strtol(val, NULL, 10);
        int sel = hz == 12 ? 1 : hz == 96 ? 2 : hz == 384 ? 3 : hz == 0 ? 0 : -1;
        if (sel < 0) { proto_error("hpf 12|96|384|0"); return; }
        if (!app.adc_ok) { proto_error("hpf: no ADC"); return; }
        if (!tlv320_set_hpf(&app.adc, sel)) { proto_error("hpf: I2C write failed"); return; }
        app.hpf_sel = sel;
        ack_uint("hpf", (uint32_t)tlv320_hpf_hz(sel));
    } else if (streq_ci(key, "ch2")) {
        bool on = streq_ci(val, "1") || streq_ci(val, "on") || streq_ci(val, "true");
        if (!app.adc_ok) { proto_error("ch2: no ADC"); return; }
        bool ok = true;
        if (on) {
            ok &= tlv320_config_channel(&app.adc, 2, false, 2500);
            ok &= tlv320_set_gain(&app.adc, 2, app.gain_db);
            ok &= tlv320_set_volume(&app.adc, 2, 0);
        }
        ok &= tlv320_enable_channels(&app.adc, true, on);
        if (!ok) { proto_error("ch2: I2C write failed"); return; }
        app.ch2 = on;
        ack_bool("ch2", on);
    } else {
        proto_error("unknown key %s", key);
    }
}

static void cmd_dump(void) {
    msg_len = 0;
    msg_printf("{\"type\":\"regs\",\"regs\":{");
    if (app.adc_ok) {
        for (int r = 0; r < 0x78; r++) {
            uint8_t v = 0;
            tlv320_read(&app.adc, (uint8_t)r, &v);
            msg_printf("%s\"0x%02X\":%u", r ? "," : "", r, v);
        }
    }
    msg_printf("}}\n");
    send_msg(USB_MSG_CTRL);
}

void proto_handle_line(const char *line) {
    app.last_cmd_us = time_us_64();
    char buf[256];
    strncpy(buf, line, sizeof buf - 1);
    buf[sizeof buf - 1] = 0;
    char *save = NULL;
    const char *tok[4] = {NULL, NULL, NULL, NULL};
    int nt = 0;
    for (char *t = strtok_r(buf, " \t", &save); t && nt < 4; t = strtok_r(NULL, " \t", &save)) tok[nt++] = t;
    if (!nt) return;
    const char *cmd = tok[0];

    if (streq_ci(cmd, "ping")) {
        return;
    } else if (streq_ci(cmd, "id")) {
        proto_send_id();
    } else if (streq_ci(cmd, "status")) {
        app_refresh_status();
        proto_send_status();
    } else if (streq_ci(cmd, "selftest")) {
        selftest_run();
    } else if (streq_ci(cmd, "start")) {
        app.streaming = true;
        ack_bool("stream", true);
    } else if (streq_ci(cmd, "stop")) {
        app.streaming = false;
        ack_bool("stream", false);
    } else if (streq_ci(cmd, "light")) {
        bool on = nt > 1 && (streq_ci(tok[1], "on") || streq_ci(tok[1], "1") || streq_ci(tok[1], "true"));
        app_set_light(on);
        ack_bool("light", app.light);
    } else if (streq_ci(cmd, "set") && nt >= 3) {
        cmd_set(tok[1], tok[2]);
    } else if (streq_ci(cmd, "dump")) {
        cmd_dump();
    } else if (streq_ci(cmd, "bootsel")) {
        app_set_light(false);
        ack_bool("bootsel", true);
        usb_pump();
        sleep_ms(50);
        usb_pump();
        reset_usb_boot(0, 0);
    } else if (streq_ci(cmd, "reset")) {
        app_set_light(false);
        ack_bool("reset", true);
        usb_pump();
        sleep_ms(50);
        usb_pump();
        watchdog_reboot(0, 0, 10);
    } else {
        proto_error("unknown command: %s", line);
    }
}

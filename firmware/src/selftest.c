/*
 * selftest.c - see selftest.h. Entry names match the MicroPython firmware so the
 * GUI dialog needs no change; new entries are additive.
 */
#include "selftest.h"
#include "app.h"
#include "i2s_rx.h"
#include "lockin.h"
#include "modulator.h"
#include "protocol.h"
#include "usb_io.h"

#include <stdio.h>
#include <string.h>
#include "hardware/clocks.h"
#include "hardware/gpio.h"
#include "hardware/pio.h"
#include "hardware/watchdog.h"
#include "pico/time.h"
#include "framecount.pio.h"

_Static_assert(framecount_BCLK_GPIO == PIN_BCLK && framecount_FSYNC_GPIO == PIN_FSYNC,
               "framecount.pio watches the wrong pins");

static uint fc_offset;

void selftest_init(void) {
    fc_offset = pio_add_program(FC_PIO, &framecount_program);
    pio_sm_config c = framecount_program_get_default_config(fc_offset);
    sm_config_set_jmp_pin(&c, PIN_FSYNC);
    sm_config_set_clkdiv_int_frac(&c, 1, 0);
    pio_sm_init(FC_PIO, FC_SM, fc_offset, &c);
}

uint32_t selftest_bclk_per_frame(void) {
    pio_sm_set_enabled(FC_PIO, FC_SM, false);
    pio_sm_restart(FC_PIO, FC_SM);
    pio_sm_clear_fifos(FC_PIO, FC_SM);
    pio_sm_exec(FC_PIO, FC_SM, pio_encode_jmp(fc_offset));
    pio_sm_set_enabled(FC_PIO, FC_SM, true);
    uint64_t t0 = time_us_64();
    uint32_t n = 0;
    while (time_us_64() - t0 < 5000) {
        if (!pio_sm_is_rx_fifo_empty(FC_PIO, FC_SM)) {
            n = 2 * pio_sm_get(FC_PIO, FC_SM);
            break;
        }
    }
    pio_sm_set_enabled(FC_PIO, FC_SM, false);
    return n;
}

static int pin_levels_seen(uint pin, uint32_t duration_us) {
    int seen = 0;
    uint64_t t0 = time_us_64();
    do {
        seen |= 1 << gpio_get(pin);
    } while (time_us_64() - t0 < duration_us);
    return seen;                        /* 1 = only low, 2 = only high, 3 = both */
}

void selftest_run(void) {
    char d[200];
    uint64_t t_start = time_us_64();
    proto_selftest_begin();

    /* ADC presence and identity */
    bool ack = app.adc_ok ? tlv320_present(&app.adc) : tlv320_present(&app.adc);
    snprintf(d, sizeof d, "I2C0 address 0x%02X %s", TLV320_I2C_ADDR, ack ? "acknowledged" : "silent");
    proto_selftest_add("adc_i2c_ack", ack ? "pass" : "fail", d);
    bool reg = ack && tlv320_detect(&app.adc);
    proto_selftest_add("adc_register_test", reg ? "pass" : "fail", "page register write/read-back");
    if (app.adc_defaults_bad < 0) {
        proto_selftest_add("adc_reset_defaults", "skip", "ADC not initialised");
    } else {
        snprintf(d, sizeof d, "mismatches: %d %s", app.adc_defaults_bad, app.adc_defaults_detail);
        proto_selftest_add("adc_reset_defaults", app.adc_defaults_bad == 0 ? "pass" : "fail", d);
    }

    /* clocks and mode */
    app_refresh_status();
    const tlv320_status_t *s = &app.adc_status;
    snprintf(d, sizeof d, "fs=%s ratio=%d", s->fs ? s->fs : "?", s->bclk_ratio);
    proto_selftest_add("adc_clock_detect", s->clock_ok ? "pass" : "fail", d);
    bool active = s->mode && strstr(s->mode, "channel(s) on");
    snprintf(d, sizeof d, "mode=%s ch1=%s", s->mode ? s->mode : "?", s->ch1_on ? "on" : "off");
    proto_selftest_add("adc_active_mode", active ? "pass" : "fail", d);

    uint32_t sys_hz = clock_get_hz(clk_sys);
    snprintf(d, sizeof d, "%lu Hz (%s), BCLK %d Hz, fs %d Hz from integer dividers",
             (unsigned long)sys_hz, app.clk_ok ? "as configured" : "NOT the configured clock", BCLK_HZ, FS_HZ);
    proto_selftest_add("sysclk", sys_hz == SYS_CLK_HZ ? "pass" : "fail", d);

    int seen = pin_levels_seen(PIN_BCLK, 400);
    snprintf(d, sizeof d, "I2S bit clock on GP%d %s", PIN_BCLK, seen == 3 ? "toggling" : "stuck");
    proto_selftest_add("bclk_toggling", seen == 3 ? "pass" : "fail", d);
    seen = pin_levels_seen(PIN_FSYNC, 400);
    snprintf(d, sizeof d, "I2S word select on GP%d %s", PIN_FSYNC, seen == 3 ? "toggling" : "stuck");
    proto_selftest_add("fsync_toggling", seen == 3 ? "pass" : "fail", d);
    uint32_t bpf = selftest_bclk_per_frame();
    snprintf(d, sizeof d, "%lu bit clocks per frame (modulator assumes %d)", (unsigned long)bpf, BCLK_PER_FRAME);
    proto_selftest_add("bclk_per_frame", bpf == BCLK_PER_FRAME ? "pass" : "fail", d);

    uint32_t per = i2s_block_period_us();
    if (i2s_blocks_completed() < 2) {
        proto_selftest_add("block_timing", "skip", "fewer than two blocks received");
    } else {
        int32_t err = (int32_t)per - 100000;
        snprintf(d, sizeof d, "%lu us per 4800-frame block (expected 100000 +- 2)", (unsigned long)per);
        proto_selftest_add("block_timing", (err >= -2 && err <= 2) ? "pass" : "fail", d);
    }

    bool lm = lockin_selftest(d, sizeof d);
    proto_selftest_add("lockin_math", lm ? "pass" : "fail", d);

    snprintf(d, sizeof d, "%lu DMA overruns, %lu blocks skipped by the main loop",
             (unsigned long)i2s_overruns(), (unsigned long)app.blocks_skipped);
    proto_selftest_add("dma_overruns", i2s_overruns() == 0 && app.blocks_skipped == 0 ? "pass" : "warn", d);
    snprintf(d, sizeof d, "dropped %lu bursts, %lu data/log messages (USB slower than the stream)",
             (unsigned long)usb_drop_burst(), (unsigned long)usb_drop_data());
    proto_selftest_add("usb_tx", usb_drop_burst() == 0 && usb_drop_data() == 0 ? "pass" : "warn", d);
    snprintf(d, sizeof d, "%d ms watchdog armed, last reset: %s", WATCHDOG_MS, app.wd_reboot ? "watchdog" : "power/reset");
    proto_selftest_add("watchdog", app.wd_reboot ? "warn" : "pass", d);
    snprintf(d, sizeof d, "%lu I2C errors since boot", (unsigned long)app.adc.i2c_errors);
    proto_selftest_add("adc_i2c_errors", app.adc.i2c_errors == 0 ? "pass" : "warn", d);

    /* microphone / input */
    if (!app.have_last) {
        proto_selftest_add("mic_noise_floor", "skip", "no audio block yet");
    } else {
        double rms_db = dbfs(app.last.rms);
        const char *st;
        if (rms_db < -110) {
            st = "fail";
            snprintf(d, sizeof d, "input silent (%.1f dBFS): mic dead, unpowered or open", rms_db);
        } else if (rms_db > -20) {
            st = "warn";
            snprintf(d, sizeof d, "very loud (%.1f dBFS): overload, oscillation or hum", rms_db);
        } else {
            st = "pass";
            snprintf(d, sizeof d, "%.1f dBFS at %.1f dB gain", rms_db, app.gain_db);
        }
        proto_selftest_add("mic_noise_floor", st, d);
        snprintf(d, sizeof d, "peak %ld / 32767", (long)app.last.peak16);
        proto_selftest_add("adc_overload", app.last.peak16 < 32000 ? "pass" : "warn", d);
        snprintf(d, sizeof d, "mean %.3f %% of full scale", 100.0 * app.last.dc);
        proto_selftest_add("adc_dc_offset", app.last.dc > -0.01 && app.last.dc < 0.01 ? "pass" : "warn", d);
        if (app.ch2) {
            double a = dbfs(app.last.rms), c2 = dbfs(app.rms_ch2);
            snprintf(d, sizeof d, "ch1 (mic) %.1f dBFS, ch2 (open) %.1f dBFS at %.1f dB gain", a, c2, app.gain_db);
            proto_selftest_add("adc_ch2_compare", c2 < -60 ? "pass" : "warn", d);
        }
    }

    /* modulator */
    seen = pin_levels_seen(PIN_MOD, 12000);          /* one period is at most 5 ms */
    if (app.light) {
        snprintf(d, sizeof d, "GP%d %s at %.3f Hz", PIN_MOD, seen == 3 ? "toggling" : "stuck", app.freq);
        proto_selftest_add("mod_pin", seen == 3 ? "pass" : "fail", d);
    } else {
        snprintf(d, sizeof d, "GP%d %s with the light off", PIN_MOD, seen == 1 ? "low" : "NOT low");
        proto_selftest_add("mod_pin", seen == 1 ? "pass" : "fail", d);
    }
    snprintf(d, sizeof d, "PIO %s, P = %lu bit clocks (%.3f Hz), 50 %% duty, sync %s",
             modulator_running() ? "running" : "idle", (unsigned long)app.p_bclk, app.freq, modulator_sync_str());
    proto_selftest_add("modulator_state", "pass", d);
    if (app.light) {
        const char *sync = modulator_sync_str();
        bool good = strcmp(sync, "frame0") == 0 || strcmp(sync, "frame1") == 0;
        snprintf(d, sizeof d, "last start on %s of block %lu", sync, (unsigned long)app.mod_seq);
        proto_selftest_add("mod_sync", good ? "pass" : "warn", d);
    } else {
        proto_selftest_add("mod_sync", "skip", "light is off");
    }
    proto_selftest_add("light_functional", "skip", "run the light on/off check from the PC");

    uint32_t dur_ms = (uint32_t)((time_us_64() - t_start) / 1000);
    snprintf(d, sizeof d, "%lu ms (the main loop must stay under 100 ms to keep every block)", (unsigned long)dur_ms);
    proto_selftest_add("selftest_duration", dur_ms < 80 ? "pass" : "warn", d);
    proto_selftest_end();
}

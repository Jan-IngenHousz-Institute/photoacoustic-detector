/*
 * main.c - photoacoustic detector firmware for the RP2040 Pico (Pico SDK).
 *
 * Hardware (see kicad/ and board.h):
 *   ADC TLV320ADC6120   I2C0 GP4/GP5, I2S GP6 BCLK, GP7 FSYNC, GP8 SDOUT (Pico is master)
 *   LED current source  GP0 (Pico pin 1) control input, on/off: high = LED on
 *
 * The LED is square-wave modulated by a PIO state machine that counts I2S bit
 * clocks (modulator.c), f_mod = 3.072 MHz / P, and it is started on a known
 * sample frame so the lock-in phase is referenced to the LED rising edge. The
 * ADC stream (24-bit samples in 32-bit slots, 48000.000 Hz from integer clock
 * dividers) arrives by DMA in 100 ms blocks (i2s_rx.c). Every block the lock-in
 * I/Q at f_mod is reported as one JSON line and, with `burst` enabled, the raw
 * channel-1 samples as int16 base64 (protocol.c) over USB CDC (usb_io.c).
 *
 * Safety: GP0 is driven low first thing at boot; the light is switched off
 * when the PC stops sending commands (heartbeat timer), when the USB host goes
 * away (DTR), on panic / hard fault, and a 1 s watchdog reboots a stuck
 * firmware (safety.c).
 */
#include <string.h>
#include "pico/stdlib.h"
#include "hardware/clocks.h"
#include "hardware/gpio.h"
#include "hardware/i2c.h"
#include "hardware/watchdog.h"

#include "app.h"
#include "i2s_rx.h"
#include "lockin.h"
#include "modulator.h"
#include "protocol.h"
#include "safety.h"
#include "selftest.h"
#include "usb_io.h"

app_t app;

static uint32_t clamp_p(uint32_t p) {
    return p < P_MIN ? P_MIN : (p > P_MAX ? P_MAX : p);
}

void app_set_period(uint32_t p) {
    p = clamp_p(p);
    app.p_bclk = p;
    app.freq = (double)BCLK_HZ / p;
    if (app.light) modulator_request_start(p);        /* restart with the new period at the next block */
}

void app_set_light(bool on) {
    if (on) {
        app.light = true;
        modulator_request_start(app.p_bclk);
    } else {
        modulator_off();
        app.light = false;
    }
    gpio_put(PIN_LED, app.light);
}

void app_refresh_status(void) {
    if (app.adc_ok) {
        tlv320_status(&app.adc, &app.adc_status);
    } else {
        memset(&app.adc_status, 0, sizeof app.adc_status);
        app.adc_status.mode = "no adc";
        app.adc_status.fs = "-";
        app.adc_status.bclk_ratio = -1;
    }
}

static void init_adc(void) {
    i2c_init(i2c0, 400 * 1000);
    gpio_set_function(PIN_ADC_SDA, GPIO_FUNC_I2C);
    gpio_set_function(PIN_ADC_SCL, GPIO_FUNC_I2C);
    tlv320_init(&app.adc, i2c0);
    app.adc_defaults_bad = -1;
    app.adc_ok = tlv320_detect(&app.adc);
    if (!app.adc_ok) {
        proto_log("error", "TLV320ADC6120 not found on I2C0");
        return;
    }
    tlv320_reset(&app.adc);
    app.adc_defaults_bad = tlv320_verify(&app.adc, app.adc_defaults_detail, sizeof app.adc_defaults_detail);
    if (!tlv320_begin(&app.adc, app.gain_db, app.ch2, app.hpf_sel))
        proto_log("error", "ADC configuration failed (I2C)");
}

/* Right-slot RMS for the channel-2 comparison (self-test), cheap enough. */
static double right_rms(const int32_t *frames) {
    double ss = 0;
    for (uint32_t n = 0; n < BLOCK; n++) {
        double v = (double)(frames[2 * n + 1] >> 8);
        ss += v * v;
    }
    return __builtin_sqrt(ss / BLOCK) / 8388608.0;
}

static void process_block(uint32_t seq, const int32_t *frames, uint64_t t_us) {
    lockin_t *li = &app.li;

    /* A modulator (re)start applies from its start block on: same period as the
     * light, reference phase 0 on the LED rising edge of that block's frame m. */
    uint32_t sseq, sp, scount;
    int sframe;
    if (modulator_start_info(&sseq, &sp, &sframe, &scount) && scount != app.mod_count_applied && sseq <= seq) {
        app.mod_count_applied = scount;
        app.mod_seq = sseq;
        lockin_set_period(li, sp);
        if (sframe >= 0) {
            double late = (double)(seq - sseq) * BLOCK;        /* blocks skipped since the start */
            lockin_set_phase(li, (late * BCLK_PER_FRAME - (sframe * BCLK_PER_FRAME + MOD_EDGE_BCLK)) / sp);
        }
    } else if (!app.light && li->p != app.p_bclk) {
        lockin_set_period(li, app.p_bclk);                     /* free-running reference follows the setting */
    }

    lockin_block_begin(li);
    for (uint32_t k = 0; k < BLOCK; k += CHUNK) {
        lockin_process(li, frames + 2 * k, CHUNK);
        usb_pump();                                            /* keep USB flowing during the 10 ms of math */
        watchdog_update();
    }
    lockin_block_result(li, &app.last);
    app.have_last = true;
    if (app.ch2) app.rms_ch2 = right_rms(frames);
    app.seq = seq;

    if (seq % 10 == 0) app_refresh_status();
    if (app.streaming) {
        proto_send_data(seq, t_us);
        if (app.burst_every && seq % app.burst_every == 0) proto_send_burst(seq, frames);
    }
    if (!app.light) gpio_put(PIN_LED, (seq % 10) < 5);
}

int main(void) {
    /* LED current source off before anything else */
    gpio_init(PIN_MOD);
    gpio_set_dir(PIN_MOD, GPIO_OUT);
    gpio_put(PIN_MOD, 0);

    app.wd_reboot = watchdog_enable_caused_reboot();     /* a timeout, not the bootloader's reboot */
    app.clk_ok = clock_get_hz(clk_sys) == SYS_CLK_HZ;      /* set up by the SDK runtime from CMakeLists.txt */

    gpio_init(PIN_LED);
    gpio_set_dir(PIN_LED, GPIO_OUT);

    app.p_bclk = P_DEFAULT;
    app.freq = (double)BCLK_HZ / P_DEFAULT;
    app.gain_db = DEFAULT_GAIN_DB;
    app.streaming = true;
    app.burst_every = 1;
    app.heartbeat_s = DEFAULT_HEARTBEAT_S;
    app.hpf_sel = DEFAULT_HPF_SEL;
    app.t0_us = time_us_64();
    app.last_cmd_us = app.t0_us;

    lockin_tables_init();
    lockin_init(&app.li, app.p_bclk);

    usb_init(proto_handle_line);           /* first, so the boot messages below are kept */
    if (!app.clk_ok) proto_log("error", "system clock is not 76.8 MHz: sample rate and f_mod are wrong");
    if (app.wd_reboot) proto_log("warn", "rebooted by the watchdog");

    init_adc();
    i2s_init();
    modulator_init();
    selftest_init();
    safety_init();

    sleep_ms(50);                          /* ADC PLL lock */
    app_refresh_status();
    proto_send_id();

    while (true) {
        watchdog_update();
        usb_pump();
        if (app.heartbeat_lost) {
            app.heartbeat_lost = false;
            gpio_put(PIN_LED, 0);
            proto_log("warn", "heartbeat lost: light switched off");
        }
        uint32_t seq;
        const int32_t *frames;
        uint64_t t_us;
        bool skipped;
        if (i2s_block_take(&seq, &frames, &t_us, &skipped)) {
            if (skipped) {
                app.blocks_skipped++;
                proto_log("warn", "main loop fell behind: block(s) skipped before seq %lu", (unsigned long)seq);
            }
            process_block(seq, frames, t_us);
            i2s_block_release();
        }
    }
}

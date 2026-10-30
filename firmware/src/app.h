/*
 * app.h - firmware state shared between the main loop, the protocol handler,
 * the self-test and the safety hooks.
 */
#pragma once
#include <stdbool.h>
#include <stdint.h>
#include "board.h"
#include "lockin.h"
#include "tlv320adc6120.h"

typedef struct {
    /* settings */
    uint32_t p_bclk;                /* modulation period in bit clocks */
    double freq;                    /* BCLK_HZ / p_bclk */
    float gain_db;
    bool light;                     /* modulation requested on */
    bool streaming;
    uint32_t burst_every;
    uint32_t heartbeat_s;
    int hpf_sel;
    bool ch2;
    /* state */
    volatile uint64_t last_cmd_us;
    uint32_t seq;                   /* last block processed */
    bool adc_ok;
    int adc_defaults_bad;           /* -1 = not checked */
    char adc_defaults_detail[96];
    tlv320_t adc;
    tlv320_status_t adc_status;
    lockin_t li;
    lockin_result_t last;
    bool have_last;
    double rms_ch2;                 /* right slot RMS of the last block (ch2 enabled) */
    uint32_t mod_count_applied;     /* modulator start already applied to the lock-in */
    uint32_t mod_seq;               /* block the current modulation phase origin refers to */
    bool wd_reboot;
    bool clk_ok;
    uint64_t t0_us;
    uint32_t blocks_skipped;
    volatile bool heartbeat_lost;   /* set by the safety timer, reported by the main loop */
} app_t;

extern app_t app;

void app_set_light(bool on);
void app_set_period(uint32_t p);
void app_refresh_status(void);

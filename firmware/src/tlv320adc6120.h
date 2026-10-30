/*
 * tlv320adc6120.h - I2C driver for the TI TLV320ADC6120 audio ADC (page 0).
 * Fixed 7-bit address 0x4E (0x4C answers only in broadcast mode). Port of
 * firmware-micropython/TLV320.py with timeouts and error counting.
 */
#pragma once
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#include "hardware/i2c.h"

#define TLV320_I2C_ADDR 0x4E

enum {
    TLV_PAGE_CFG = 0x00, TLV_SW_RESET = 0x01, TLV_SLEEP_CFG = 0x02, TLV_SHDN_CFG = 0x05,
    TLV_ASI_CFG0 = 0x07, TLV_ASI_CFG1 = 0x08, TLV_ASI_CFG2 = 0x09, TLV_ASI_CH1 = 0x0B,
    TLV_ASI_CH2 = 0x0C, TLV_MST_CFG0 = 0x13, TLV_MST_CFG1 = 0x14, TLV_ASI_STS = 0x15,
    TLV_CLK_SRC = 0x16, TLV_GPIO_CFG0 = 0x21, TLV_GPO_CFG0 = 0x22, TLV_GPO_VAL = 0x29,
    TLV_GPIO_MON = 0x2A, TLV_INT_CFG = 0x32, TLV_INT_MASK0 = 0x33, TLV_INT_LTCH0 = 0x36,
    TLV_BIAS_CFG = 0x3B, TLV_CH1_CFG0 = 0x3C, TLV_CH1_CFG1 = 0x3D, TLV_CH1_CFG2 = 0x3E,
    TLV_CH1_CFG3 = 0x3F, TLV_CH1_CFG4 = 0x40, TLV_CH2_CFG0 = 0x41, TLV_CH2_CFG1 = 0x42,
    TLV_CH2_CFG2 = 0x43, TLV_CH2_CFG3 = 0x44, TLV_CH2_CFG4 = 0x45, TLV_DSP_CFG0 = 0x6B,
    TLV_DSP_CFG1 = 0x6C, TLV_DRE_CFG0 = 0x6D, TLV_AGC_CFG0 = 0x70, TLV_GAIN_CFG = 0x71,
    TLV_IN_CH_EN = 0x73, TLV_ASI_OUT_CH_EN = 0x74, TLV_PWR_CFG = 0x75, TLV_DEV_STS0 = 0x76,
    TLV_DEV_STS1 = 0x77,
};

typedef struct {
    i2c_inst_t *i2c;
    uint8_t addr;
    uint32_t i2c_errors;
} tlv320_t;

typedef struct {
    bool clock_ok, ch1_on, ch2_on;
    const char *mode;       /* decoded DEV_STS1 */
    const char *fs;         /* detected sample rate, "48k" ... */
    int bclk_ratio;         /* detected BCLK/FSYNC ratio, -1 = invalid */
    uint8_t int_latch;
} tlv320_status_t;

void tlv320_init(tlv320_t *d, i2c_inst_t *i2c);
bool tlv320_write(tlv320_t *d, uint8_t reg, uint8_t val);
bool tlv320_read(tlv320_t *d, uint8_t reg, uint8_t *val);
bool tlv320_update(tlv320_t *d, uint8_t reg, uint8_t mask, uint8_t val);
bool tlv320_present(tlv320_t *d);
bool tlv320_detect(tlv320_t *d);
int tlv320_verify(tlv320_t *d, char *detail, size_t len);   /* mismatches vs reset defaults */
void tlv320_reset(tlv320_t *d);
void tlv320_wake(tlv320_t *d, bool internal_areg);
bool tlv320_config_channel(tlv320_t *d, int ch, bool line_in, int impedance_ohm);
bool tlv320_set_gain(tlv320_t *d, int ch, float db);          /* 0 .. 42 dB, 0.5 dB steps */
bool tlv320_set_volume(tlv320_t *d, int ch, float db);        /* digital, -100 .. +27 dB */
bool tlv320_set_hpf(tlv320_t *d, int sel);                    /* 0 custom(all-pass), 1 12 Hz, 2 96 Hz, 3 384 Hz */
bool tlv320_enable_channels(tlv320_t *d, bool ch1, bool ch2);
bool tlv320_power_up(tlv320_t *d, bool micbias, bool adc, bool pll);
bool tlv320_begin(tlv320_t *d, float gain_db, bool ch2, int hpf_sel);
bool tlv320_status(tlv320_t *d, tlv320_status_t *s);
int tlv320_hpf_hz(int sel);

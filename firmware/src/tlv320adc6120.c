/*
 * tlv320adc6120.c - see tlv320adc6120.h. Register map: TI SBASA92A, 8.6.2.
 */
#include "tlv320adc6120.h"

#include <math.h>
#include <stdio.h>
#include "pico/time.h"

#define I2C_TIMEOUT_US 2000

static const struct { uint8_t reg, val; } reset_defaults[] = {
    {TLV_SLEEP_CFG, 0x00}, {TLV_SHDN_CFG, 0x05}, {TLV_ASI_CFG0, 0x30}, {TLV_MST_CFG1, 0x48},
    {TLV_CLK_SRC, 0x10}, {TLV_GPIO_CFG0, 0x22}, {TLV_CH1_CFG2, 0xC9}, {TLV_CH1_CFG3, 0x80},
    {TLV_CH2_CFG2, 0xC9}, {TLV_DSP_CFG0, 0x01}, {TLV_DSP_CFG1, 0x40}, {TLV_IN_CH_EN, 0xC0},
    {TLV_DEV_STS1, 0x80},
};

static const char *fs_names[] = {"8k", "16k", "24k", "32k", "48k", "96k", "192k", "384k", "768k"};
static const int ratio_values[] = {16, 24, 32, 48, 64, 96, 128, 192, 256, 384, 512, 1024, 2048};

void tlv320_init(tlv320_t *d, i2c_inst_t *i2c) {
    d->i2c = i2c;
    d->addr = TLV320_I2C_ADDR;
    d->i2c_errors = 0;
}

bool tlv320_write(tlv320_t *d, uint8_t reg, uint8_t val) {
    uint8_t buf[2] = {reg, val};
    if (i2c_write_timeout_us(d->i2c, d->addr, buf, 2, false, I2C_TIMEOUT_US) != 2) {
        d->i2c_errors++;
        return false;
    }
    return true;
}

bool tlv320_read(tlv320_t *d, uint8_t reg, uint8_t *val) {
    int r = i2c_write_timeout_us(d->i2c, d->addr, &reg, 1, true, I2C_TIMEOUT_US);
    if (r == 1) r = i2c_read_timeout_us(d->i2c, d->addr, val, 1, false, I2C_TIMEOUT_US);
    if (r != 1) {
        d->i2c_errors++;
        return false;
    }
    return true;
}

bool tlv320_update(tlv320_t *d, uint8_t reg, uint8_t mask, uint8_t val) {
    uint8_t cur;
    if (!tlv320_read(d, reg, &cur)) return false;
    uint8_t nv = (uint8_t)((cur & ~mask) | (val & mask));
    if (nv == cur) return true;
    return tlv320_write(d, reg, nv);
}

bool tlv320_present(tlv320_t *d) {
    uint8_t v;
    int r = i2c_read_timeout_us(d->i2c, d->addr, &v, 1, false, I2C_TIMEOUT_US);
    return r == 1;
}

/* The part has no ID register: PAGE_CFG must read back what was written. */
bool tlv320_detect(tlv320_t *d) {
    uint8_t v;
    if (!tlv320_present(d)) return false;
    if (!tlv320_write(d, TLV_PAGE_CFG, 1) || !tlv320_read(d, TLV_PAGE_CFG, &v) || v != 1) {
        tlv320_write(d, TLV_PAGE_CFG, 0);
        return false;
    }
    if (!tlv320_write(d, TLV_PAGE_CFG, 0) || !tlv320_read(d, TLV_PAGE_CFG, &v) || v != 0) return false;
    return true;
}

int tlv320_verify(tlv320_t *d, char *detail, size_t len) {
    int bad = 0;
    size_t pos = 0;
    if (len) detail[0] = 0;
    for (size_t k = 0; k < sizeof reset_defaults / sizeof reset_defaults[0]; k++) {
        uint8_t got;
        if (!tlv320_read(d, reset_defaults[k].reg, &got)) {
            bad++;
            continue;
        }
        if (got != reset_defaults[k].val) {
            bad++;
            if (pos < len) {
                int n = snprintf(detail + pos, len - pos, "%sR%d=0x%02X(exp 0x%02X)", pos ? " " : "",
                                 reset_defaults[k].reg, got, reset_defaults[k].val);
                if (n > 0) pos += (size_t)n;
            }
        }
    }
    return bad;
}

void tlv320_reset(tlv320_t *d) {
    tlv320_write(d, TLV_PAGE_CFG, 0);
    tlv320_write(d, TLV_SW_RESET, 0x01);
    sleep_ms(2);                               /* datasheet: >= 1 ms */
}

void tlv320_wake(tlv320_t *d, bool internal_areg) {
    tlv320_write(d, TLV_SLEEP_CFG, (uint8_t)(0x01 | (internal_areg ? 0x80 : 0x00)));
    sleep_ms(5);                               /* >= 1 ms wake-up, VREF quick charge ~3.5 ms */
}

bool tlv320_config_channel(tlv320_t *d, int ch, bool line_in, int impedance_ohm) {
    uint8_t imp = impedance_ohm >= 20000 ? 2 : (impedance_ohm >= 10000 ? 1 : 0);
    uint8_t val = (uint8_t)((imp << 2) | (line_in ? 0x80 : 0));
    return tlv320_write(d, ch == 1 ? TLV_CH1_CFG0 : TLV_CH2_CFG0, val);
}

bool tlv320_set_gain(tlv320_t *d, int ch, float db) {
    int code = (int)lroundf(db * 2.0f);
    if (code < 0 || code > 84) return false;
    return tlv320_write(d, ch == 1 ? TLV_CH1_CFG1 : TLV_CH2_CFG1, (uint8_t)(code << 1));
}

bool tlv320_set_volume(tlv320_t *d, int ch, float db) {
    int code = 201 + (int)lroundf(db * 2.0f);
    if (code < 1 || code > 255) return false;
    return tlv320_write(d, ch == 1 ? TLV_CH1_CFG2 : TLV_CH2_CFG2, (uint8_t)code);
}

bool tlv320_set_hpf(tlv320_t *d, int sel) {
    return tlv320_update(d, TLV_DSP_CFG0, 0x03, (uint8_t)(sel & 0x03));
}

int tlv320_hpf_hz(int sel) {
    switch (sel & 3) {
        case 1: return 12;
        case 2: return 96;
        case 3: return 384;
        default: return 0;
    }
}

bool tlv320_enable_channels(tlv320_t *d, bool ch1, bool ch2) {
    uint8_t val = (uint8_t)((ch1 ? 0x80 : 0) | (ch2 ? 0x40 : 0));
    return tlv320_write(d, TLV_IN_CH_EN, val) && tlv320_write(d, TLV_ASI_OUT_CH_EN, val);
}

bool tlv320_power_up(tlv320_t *d, bool micbias, bool adc, bool pll) {
    uint8_t val = (uint8_t)((micbias ? 0x80 : 0) | (adc ? 0x40 : 0) | (pll ? 0x20 : 0));
    return tlv320_write(d, TLV_PWR_CFG, val);
}

/* Full bring-up for this board: reset -> wake (internal AREG, AVDD = 3.3 V) ->
 * I2S slave, 32-bit slots (24-bit data in the top bits) -> CH1 differential
 * microphone input, 2.5 k -> digital volume 0 dB -> HPF -> enable -> power up.
 * The Pico's I2S clocks may start before or after this; the ADC locks its PLL
 * to BCLK/FSYNC automatically. */
bool tlv320_begin(tlv320_t *d, float gain_db, bool ch2, int hpf_sel) {
    bool ok = true;
    tlv320_reset(d);
    tlv320_wake(d, true);
    ok &= tlv320_write(d, TLV_ASI_CFG0, 0x70);   /* I2S, 32-bit word length */
    ok &= tlv320_write(d, TLV_MST_CFG0, 0x00);   /* slave, auto clock config with PLL */
    ok &= tlv320_config_channel(d, 1, false, 2500);
    ok &= tlv320_set_gain(d, 1, gain_db);
    ok &= tlv320_set_volume(d, 1, 0);
    if (ch2) {
        ok &= tlv320_config_channel(d, 2, false, 2500);
        ok &= tlv320_set_gain(d, 2, gain_db);
        ok &= tlv320_set_volume(d, 2, 0);
    }
    ok &= tlv320_set_hpf(d, hpf_sel);
    ok &= tlv320_enable_channels(d, true, ch2);
    ok &= tlv320_power_up(d, true, true, true);
    return ok;
}

bool tlv320_status(tlv320_t *d, tlv320_status_t *s) {
    uint8_t sts0, sts1, asi, latch;
    if (!tlv320_read(d, TLV_DEV_STS0, &sts0) || !tlv320_read(d, TLV_DEV_STS1, &sts1) ||
        !tlv320_read(d, TLV_ASI_STS, &asi) || !tlv320_read(d, TLV_INT_LTCH0, &latch)) {
        s->clock_ok = false;
        s->mode = "i2c error";
        s->fs = "?";
        s->bclk_ratio = -1;
        return false;
    }
    uint8_t mode = sts1 >> 5, rate = asi >> 4, ratio = asi & 0x0F;
    s->ch1_on = (sts0 & 0x80) != 0;
    s->ch2_on = (sts0 & 0x40) != 0;
    s->mode = mode == 4 ? "sleep/shutdown" : mode == 6 ? "active, channels off" :
              mode == 7 ? "active, channel(s) on" : "unknown";
    s->fs = rate < 9 ? fs_names[rate] : "invalid";
    s->bclk_ratio = ratio < 13 ? ratio_values[ratio] : -1;
    s->clock_ok = rate != 0xF && ratio != 0xF;
    s->int_latch = latch;
    return true;
}

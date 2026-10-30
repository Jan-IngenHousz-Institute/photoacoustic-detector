/*
 * lockin.c - block lock-in on 24-bit samples, exact integer arithmetic.
 *
 * Reference: 4096-entry sine/cosine tables (amplitude 2^23) with linear
 * interpolation. The phase advances 64 * 4096 / P table entries per sample;
 * the fractional part is carried as an exact rational (idx, rem / P), so the
 * reference never drifts against the modulator, which counts the same bit
 * clocks. Products sample (24 bit) x reference (24 bit) are accumulated in
 * int64 (< 2^60 over a block); the DC leakage is removed exactly at the end:
 *   I = sum(v c) - mean(v) sum(c),  Q = sum(v s) - mean(v) sum(s).
 */
#include "lockin.h"
#include "board.h"

#include <math.h>
#include <stdio.h>
#include <string.h>

#define PI          3.14159265358979323846
#define TABLE_AMP   8388608.0   /* 2^23 */
#define SAMPLE_FS   8388608.0   /* 2^23: 24-bit samples, |v| < 2^23 */
#define STEP        (BCLK_PER_FRAME * TABLE_LEN)   /* table entries per sample = STEP / p */

static int32_t tsin[TABLE_LEN], tcos[TABLE_LEN];
static int16_t dsin[TABLE_LEN], dcos[TABLE_LEN];   /* next entry - this entry (|d| < 12869) */

void lockin_tables_init(void) {
    for (int k = 0; k < TABLE_LEN; k++) {
        tsin[k] = (int32_t)llround(TABLE_AMP * sin(2.0 * PI * k / TABLE_LEN));
        tcos[k] = (int32_t)llround(TABLE_AMP * cos(2.0 * PI * k / TABLE_LEN));
    }
    for (int k = 0; k < TABLE_LEN; k++) {
        int kn = (k + 1) & (TABLE_LEN - 1);
        dsin[k] = (int16_t)(tsin[kn] - tsin[k]);
        dcos[k] = (int16_t)(tcos[kn] - tcos[k]);
    }
}

void lockin_set_period(lockin_t *li, uint32_t p) {
    if (p < 2) p = 2;
    if (li->p && li->p != p) {
        /* keep the phase: rem is in units of 1/p */
        li->rem = (uint32_t)(((uint64_t)li->rem * p) / li->p);
    }
    li->p = p;
    li->q = STEP / p;
    li->r = STEP % p;
    li->inv = (1u << 18) / p;
    if (li->rem >= p) li->rem = p - 1;
}

void lockin_init(lockin_t *li, uint32_t p) {
    memset(li, 0, sizeof *li);
    lockin_set_period(li, p);
}

double lockin_phase(const lockin_t *li) {
    return (li->idx + (double)li->rem / li->p) / TABLE_LEN;
}

void lockin_set_phase(lockin_t *li, double cycles) {
    cycles -= floor(cycles);
    double e = cycles * TABLE_LEN;
    uint32_t idx = (uint32_t)floor(e);
    uint32_t rem = (uint32_t)llround((e - idx) * li->p);
    if (rem >= li->p) { rem -= li->p; idx++; }
    li->idx = idx & (TABLE_LEN - 1);
    li->rem = rem;
}

void lockin_block_begin(lockin_t *li) {
    li->ph0 = lockin_phase(li);
    li->sum_vc = li->sum_vs = li->sum_vv = li->sum_v = li->sum_c = li->sum_s = 0;
    li->peak = 0;
    li->n = 0;
}

void lockin_process(lockin_t *li, const int32_t *frames, uint32_t nframes) {
    uint32_t idx = li->idx, rem = li->rem;
    const uint32_t q = li->q, r = li->r, p = li->p, inv = li->inv;
    int64_t svc = li->sum_vc, svs = li->sum_vs, svv = li->sum_vv;
    int64_t sv = li->sum_v, sc = li->sum_c, ss = li->sum_s;
    int32_t peak = li->peak;

    for (uint32_t n = 0; n < nframes; n++) {
        int32_t v = frames[2 * n] >> 8;                      /* 24-bit sample, left slot */
        int32_t frac = (int32_t)((rem * inv) >> 2);          /* 16-bit fraction of an entry */
        int32_t c = tcos[idx] + ((dcos[idx] * frac) >> 16);
        int32_t s = tsin[idx] + ((dsin[idx] * frac) >> 16);
        svc += (int64_t)v * c;
        svs += (int64_t)v * s;
        svv += (int64_t)v * v;
        sv += v;
        sc += c;
        ss += s;
        int32_t a = v < 0 ? -v : v;
        if (a > peak) peak = a;
        idx += q;
        rem += r;
        if (rem >= p) { rem -= p; idx++; }
        idx &= TABLE_LEN - 1;
    }
    li->idx = idx; li->rem = rem;
    li->sum_vc = svc; li->sum_vs = svs; li->sum_vv = svv;
    li->sum_v = sv; li->sum_c = sc; li->sum_s = ss;
    li->peak = peak;
    li->n += nframes;
}

void lockin_block_result(const lockin_t *li, lockin_result_t *o) {
    double N = li->n ? (double)li->n : 1.0;
    double sv = (double)li->sum_v;
    double I = (double)li->sum_vc - sv * (double)li->sum_c / N;
    double Q = (double)li->sum_vs - sv * (double)li->sum_s / N;
    o->i = I * 16383.0 / 2147483648.0;     /* 16383 / (256 * 2^23) */
    o->q = Q * 16383.0 / 2147483648.0;
    o->amp = 2.0 * sqrt(I * I + Q * Q) / (TABLE_AMP * N * SAMPLE_FS);
    o->ph_deg = atan2(Q, I) * 180.0 / PI;
    o->rms = sqrt((double)li->sum_vv / N) / SAMPLE_FS;
    o->dc = sv / N / SAMPLE_FS;
    o->peak16 = li->peak >> 8;
    o->ovl = li->peak >= (int32_t)(0.98 * SAMPLE_FS);
}

double dbfs(double x) {
    return x > 0 ? 20.0 * log10(x) : -200.0;
}

/* Synthetic 24-bit sine of known amplitude and phase through the real MAC. The
 * periods divide 64 * 1200 bit clocks, so the block holds whole cycles and the
 * only errors are the table interpolation and the integer rounding. */
bool lockin_selftest(char *detail, size_t len) {
    static const uint32_t periods[] = {3072, 320, 15360};
    const uint32_t N = 240;                         /* whole cycles for all three periods, ~10 ms of double math */
    const double A = 0.5, PH = 30.0;
    double worst_a = 0, worst_p = 0;
    int32_t buf[2 * 40];
    for (size_t k = 0; k < sizeof periods / sizeof periods[0]; k++) {
        uint32_t p = periods[k];
        lockin_t li;
        lockin_init(&li, p);
        lockin_block_begin(&li);
        for (uint32_t n0 = 0; n0 < N; n0 += 40) {          /* 40 divides 240: whole cycles exactly */
            for (uint32_t j = 0; j < 40; j++) {
                /* phase in cycles with the integer part removed: the ROM cos() is only
                 * valid for |x| < 1024 rad */
                double cyc = (double)(((n0 + j) * BCLK_PER_FRAME) % p) / p;
                double th = 2.0 * PI * cyc + PH * PI / 180.0;
                buf[2 * j] = (int32_t)llround(A * SAMPLE_FS * cos(th)) * 256;   /* 24-bit data in the top bits */
                buf[2 * j + 1] = 0;
            }
            lockin_process(&li, buf, 40);
        }
        lockin_result_t r;
        lockin_block_result(&li, &r);
        double ea = fabs(r.amp / A - 1.0);
        double ep = fabs(r.ph_deg + PH);           /* cos(theta + PH) reports -PH */
        if (ea > worst_a) worst_a = ea;
        if (ep > worst_p) worst_p = ep;
    }
    bool ok = worst_a < 1e-5 && worst_p < 1e-3;
    snprintf(detail, len, "amplitude error %.2e, phase error %.2e deg (P = 3072, 320, 15360; %lu samples)",
             worst_a, worst_p, (unsigned long)N);
    return ok;
}

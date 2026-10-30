/*
 * lockin.h - block lock-in (single-bin DFT) on the 24-bit sample stream with a
 * phase-continuous reference at f_mod = BCLK_HZ / P.
 */
#pragma once
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

typedef struct {
    double i, q;        /* sums in the GUI's historic units (int16 sample x 16383 table) */
    double amp;         /* sine amplitude, fraction of full scale (1.0 = 0 dBFS peak) */
    double ph_deg;      /* atan2(q, i): a signal lagging the reference reports +phase */
    double rms;         /* broadband RMS, fraction of full scale (DC included) */
    double dc;          /* mean, fraction of full scale */
    int32_t peak16;     /* |peak| in 16-bit units (GUI divides by 32768) */
    bool ovl;           /* |peak| >= 98 % of full scale */
} lockin_result_t;

typedef struct {
    uint32_t p;                 /* modulation period in bit clocks */
    uint32_t idx, rem;          /* reference phase = (idx + rem / p) / TABLE_LEN cycles */
    uint32_t q, r, inv;         /* per-sample step idx += q, rem += r; inv = 2^18 / p */
    int64_t sum_vc, sum_vs, sum_vv, sum_v, sum_c, sum_s;
    int32_t peak;
    uint32_t n;
    double ph0;                 /* reference phase at the first sample of the block */
} lockin_t;

void lockin_tables_init(void);
void lockin_init(lockin_t *li, uint32_t p);
void lockin_set_period(lockin_t *li, uint32_t p);
void lockin_set_phase(lockin_t *li, double cycles);      /* reference phase of the next sample */
double lockin_phase(const lockin_t *li);
void lockin_block_begin(lockin_t *li);
void lockin_process(lockin_t *li, const int32_t *frames, uint32_t nframes);  /* left = even words */
void lockin_block_result(const lockin_t *li, lockin_result_t *out);
bool lockin_selftest(char *detail, size_t detail_len);
double dbfs(double x);

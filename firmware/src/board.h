/*
 * board.h - pin map, clock scheme and fixed parameters of the photoacoustic
 * detector firmware (RP2040 Pico + TLV320ADC6120 shield).
 */
#pragma once
#include <stdint.h>

#define FW_NAME         "pa-detector"
#define FW_VERSION      "1.0.1"
#define PROTO_VERSION   2

/* ---- pin map (kicad/photoacoustic-detector) ---------------------------- */
#define PIN_MOD     0       /* LED current source control, Pico pin 1: high = LED on   */
#define PIN_ADC_SDA 4       /* I2C0                                                    */
#define PIN_ADC_SCL 5
#define PIN_BCLK    6       /* I2S bit clock, Pico is master                            */
#define PIN_FSYNC   7       /* I2S word select                                          */
#define PIN_SDOUT   8       /* ADC -> Pico serial data                                  */
#define PIN_LED     25      /* Pico onboard LED                                         */

/* ---- clocks --------------------------------------------------------------
 * The system clock is 76.8 MHz = 25 PIO cycles per bit clock, so BCLK is
 * exactly 3.072 MHz and fs = 3.072 MHz / 64 = 48000.000 Hz with integer
 * dividers only (no fractional-divider jitter on the ADC's PLL input).
 * The SDK sets it up at boot from SYS_CLK_HZ / PLL_SYS_* in CMakeLists.txt
 * (VCO 1536 MHz = 12 MHz x 128, post-dividers 5 x 4).
 */
#ifndef SYS_CLK_HZ
#error "SYS_CLK_HZ must be defined (CMakeLists.txt)"
#endif
_Static_assert(SYS_CLK_HZ == 76800000, "the clock scheme assumes a 76.8 MHz system clock");
#define PIO_CYCLES_PER_BCLK 25
#define BCLK_HZ             (SYS_CLK_HZ / PIO_CYCLES_PER_BCLK)           /* 3072000 */
#define BCLK_PER_FRAME      64                                           /* 2 x 32-bit slots */
#define FS_HZ               (BCLK_HZ / BCLK_PER_FRAME)                   /* 48000 */
#define BLOCK               4800                                         /* frames per block, 100 ms */
#define BLOCK_WORDS         (2 * BLOCK)                                  /* int32 words per block */
#define CHUNK               480                                          /* frames per lock-in chunk */

/* ---- modulation: f_mod = BCLK_HZ / P, P in bit clocks ------------------ */
#define F_MIN_HZ    200.0
#define F_MAX_HZ    10000.0
#define P_MIN       308     /* ceil(3072000 / 10000) -> 9974.0 Hz, stays <= F_MAX */
#define P_MAX       15360   /* 200.000 Hz */
#define P_DEFAULT   3072    /* 1000.000 Hz */

/* LED rising-edge position inside the frame the modulator starts in, in bit
 * clocks from the frame start: the I2S program raises the frame flag at PIO
 * cycle 23 of bit period 32 and the steady-state LED edges follow the BCLK
 * rising edge (cycle 12 of a period) by 7 PIO cycles: 32 + (12 + 7) / 25. */
#define MOD_EDGE_BCLK       32.76
#define MOD_EDGE_FRAMES     (MOD_EDGE_BCLK / BCLK_PER_FRAME)

/* ---- defaults ------------------------------------------------------------ */
#define DEFAULT_HEARTBEAT_S 5
#define DEFAULT_GAIN_DB     12.0f
#define DEFAULT_HPF_SEL     1       /* 12 Hz */
#define TABLE_LEN           4096    /* lock-in reference table entries per cycle */
#define ADC_LATENCY_SAMPLES 17.1    /* linear-phase decimation group delay (datasheet) */
#define WATCHDOG_MS         1000

/* ---- PIO / DMA allocation ------------------------------------------------ */
#define I2S_PIO     pio0
#define I2S_SM      0
#define MOD_SM      1
#define FC_PIO      pio1
#define FC_SM       0

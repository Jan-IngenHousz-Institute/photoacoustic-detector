/*
 * modulator.c - see modulator.h and modulator.pio.
 *
 * Arming happens in the DMA block-boundary IRQ: the state machine is restarted
 * at its entry, GP0 is handed to the PIO (its output register already 0) and
 * the config word is pushed; the SM then clears the frame flag and starts on
 * the next one. Which frame that is follows from how many words of the new
 * block the DMA had already received: the flag of frame m is raised right
 * after word 2m (its LEFT word) is pushed. With w words received when the SM
 * cleared the flag, it starts on frame w/2 if w is even; if w is odd the flag
 * of frame (w-1)/2 was raised around the same time and the SM's program
 * counter tells whether it caught it. Normally the IRQ runs within a few
 * microseconds of the boundary and w = 0: frame 0.
 */
#include "modulator.h"
#include "board.h"

#include <stdio.h>
#include "hardware/dma.h"
#include "hardware/gpio.h"
#include "hardware/pio.h"
#include "hardware/structs/iobank0.h"
#include "hardware/structs/timer.h"
#include "hardware/sync.h"
#include "pico/platform.h"
#include "pico/time.h"
#include "modulator.pio.h"

_Static_assert(mod_square_BCLK_GPIO == PIN_BCLK, "modulator.pio watches the wrong BCLK pin");

static uint prog_offset;
static volatile bool request;
static volatile uint32_t req_p;
static volatile bool running;
static volatile uint32_t start_seq, start_p, start_count;
static volatile int start_frame = -1;
static volatile uint32_t dbg_w_entry, dbg_w_armed, dbg_arm_us;   /* arming diagnostics */

static uint32_t config_word(uint32_t p) {
    uint32_t high = p / 2, low = p - high;          /* 50 % duty, +-1 bit clock for odd p */
    return ((low - 1) << 16) | (high - 1);
}

void modulator_init(void) {
    prog_offset = pio_add_program(I2S_PIO, &mod_square_program);
    pio_sm_config c = mod_square_program_get_default_config(prog_offset);
    sm_config_set_set_pins(&c, PIN_MOD, 1);
    sm_config_set_out_shift(&c, true, false, 32);   /* shift right: H-1 first, then L-1 */
    sm_config_set_clkdiv_int_frac(&c, 1, 0);
    pio_sm_init(I2S_PIO, MOD_SM, prog_offset + mod_square_offset_entry, &c);
    pio_sm_set_consecutive_pindirs(I2S_PIO, MOD_SM, PIN_MOD, 1, true);
    pio_sm_exec(I2S_PIO, MOD_SM, pio_encode_set(pio_pins, 0));
    /* GP0 stays a SIO output driven low until the first arming */
}

void modulator_request_start(uint32_t p) {
    uint32_t s = save_and_disable_interrupts();
    req_p = p;
    request = true;
    restore_interrupts(s);
}

void modulator_off(void) {
    uint32_t s = save_and_disable_interrupts();
    request = false;
    pio_sm_set_enabled(I2S_PIO, MOD_SM, false);
    pio_sm_exec(I2S_PIO, MOD_SM, pio_encode_set(pio_pins, 0));
    gpio_set_function(PIN_MOD, GPIO_FUNC_SIO);       /* SIO output register is 0 since boot */
    running = false;
    restore_interrupts(s);
}

bool modulator_running(void) { return running; }

/* Runs from RAM inside the block IRQ: it must finish well within the 10.6 us
 * before the first word of the new block (cold XIP cache took 6..11 us). */
static void __not_in_flash_func(arm)(uint32_t word) {
    pio_sm_set_enabled(I2S_PIO, MOD_SM, false);
    pio_sm_exec(I2S_PIO, MOD_SM, pio_encode_set(pio_pins, 0));
    pio_sm_restart(I2S_PIO, MOD_SM);
    pio_sm_clear_fifos(I2S_PIO, MOD_SM);
    pio_sm_exec(I2S_PIO, MOD_SM, pio_encode_jmp(prog_offset + mod_square_offset_entry));
    /* pad -> PIO0 (what pio_gpio_init does, without the call into flash); the pad's
     * IE/OD bits were set by gpio_init() at boot and the PIO output register is 0 */
    iobank0_hw->io[PIN_MOD].ctrl = GPIO_FUNC_PIO0 << IO_BANK0_GPIO0_CTRL_FUNCSEL_LSB;
    pio_sm_set_enabled(I2S_PIO, MOD_SM, true);
    pio_sm_put(I2S_PIO, MOD_SM, word);               /* pull block completes: irq clear, wait */
}

void __not_in_flash_func(modulator_block_boundary)(uint32_t completed_blocks, int active_dma_ch) {
    if (!request) return;
    request = false;
    uint32_t p = req_p;
    dbg_w_entry = BLOCK_WORDS - dma_channel_hw_addr(active_dma_ch)->transfer_count;
    uint32_t t_arm = timer_hw->timerawl;
    arm(config_word(p));
    uint32_t w = BLOCK_WORDS - dma_channel_hw_addr(active_dma_ch)->transfer_count;
    dbg_arm_us = timer_hw->timerawl - t_arm;
    dbg_w_armed = w;
    int frame;
    if (w & 1) {
        for (volatile int i = 0; i < 30; i++) {}     /* ~1 us: the SM reacts to a flag within 50 ns */
        uint wait_pc = prog_offset + mod_square_offset_entry + 2;
        frame = pio_sm_get_pc(I2S_PIO, MOD_SM) != wait_pc ? (int)((w - 1) / 2) : (int)((w + 1) / 2);
    } else {
        frame = (int)(w / 2);
    }
    start_seq = completed_blocks + 1;                /* seq of the block being filled now */
    start_p = p;
    start_frame = frame;
    start_count++;
    running = true;
}

bool modulator_start_info(uint32_t *seq, uint32_t *p, int *frame, uint32_t *count) {
    uint32_t s = save_and_disable_interrupts();
    uint32_t c = start_count;
    *seq = start_seq;
    *p = start_p;
    *frame = start_frame;
    *count = c;
    restore_interrupts(s);
    return c != 0;
}

void modulator_arm_debug(uint32_t *w_entry, uint32_t *w_armed, uint32_t *arm_us) {
    *w_entry = dbg_w_entry;
    *w_armed = dbg_w_armed;
    *arm_us = dbg_arm_us;
}

const char *modulator_sync_str(void) {
    static char buf[12];
    if (!running) return "off";
    if (start_frame < 0) return "unknown";
    if (start_frame == 0) return "frame0";
    if (start_frame == 1) return "frame1";
    snprintf(buf, sizeof buf, "frame%d", start_frame);
    return buf;
}

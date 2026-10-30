/*
 * i2s_rx.c - see i2s_rx.h. Block j lands in bufs[j & 1]; the DMA IRQ at every
 * block boundary (highest priority) re-arms the finished channel, timestamps
 * the block and gives the modulator its chance to start (modulator.c).
 */
#include "i2s_rx.h"
#include "board.h"
#include "modulator.h"

#include "hardware/dma.h"
#include "hardware/gpio.h"
#include "hardware/irq.h"
#include "hardware/pio.h"
#include "pico/platform.h"
#include "pico/time.h"
#include "i2s_rx.pio.h"

static int32_t bufs[2][BLOCK_WORDS] __attribute__((aligned(4)));
static int dma_ch[2];
static volatile uint32_t completed;      /* blocks completed by the DMA */
static volatile uint32_t processed;      /* blocks the main loop is done with */
static volatile uint64_t t_done[2];
static volatile uint64_t last_t_us;
static volatile uint32_t period_us;
static volatile uint32_t overruns;

static void __not_in_flash_func(dma_irq_handler)(void) {
    uint64_t now = time_us_64();
    for (int k = 0; k < 2; k++) {
        if (!(dma_hw->ints0 & (1u << dma_ch[k]))) continue;
        dma_hw->ints0 = 1u << dma_ch[k];                        /* acknowledge */
        dma_channel_set_write_addr(dma_ch[k], bufs[k], false);  /* ready for its next turn */
        dma_channel_set_trans_count(dma_ch[k], BLOCK_WORDS, false);
        /* the other channel now refills the buffer of block completed-1: it must be done */
        if (processed < completed) overruns++;
        t_done[k] = now;
        if (last_t_us) period_us = (uint32_t)(now - last_t_us);
        last_t_us = now;
        completed++;
        modulator_block_boundary(completed, dma_ch[k ^ 1]);
    }
}

void i2s_init(void) {
    uint offset = pio_add_program(I2S_PIO, &i2s_rx_master_program);
    pio_sm_config c = i2s_rx_master_program_get_default_config(offset);
    sm_config_set_sideset_pins(&c, PIN_BCLK);
    sm_config_set_set_pins(&c, PIN_FSYNC, 1);
    sm_config_set_in_pins(&c, PIN_SDOUT);
    sm_config_set_in_shift(&c, false, true, 32);      /* MSB first, autopush at 32 bits */
    sm_config_set_fifo_join(&c, PIO_FIFO_JOIN_RX);
    sm_config_set_clkdiv_int_frac(&c, 1, 0);

    pio_gpio_init(I2S_PIO, PIN_BCLK);
    pio_gpio_init(I2S_PIO, PIN_FSYNC);
    pio_gpio_init(I2S_PIO, PIN_SDOUT);
    pio_sm_set_consecutive_pindirs(I2S_PIO, I2S_SM, PIN_BCLK, 2, true);
    pio_sm_set_consecutive_pindirs(I2S_PIO, I2S_SM, PIN_SDOUT, 1, false);
    /* FSYNC high, BCLK low before the first instruction: a real FSYNC falling edge */
    pio_sm_set_pins_with_mask(I2S_PIO, I2S_SM, 1u << PIN_FSYNC, (1u << PIN_FSYNC) | (1u << PIN_BCLK));
    pio_sm_init(I2S_PIO, I2S_SM, offset + i2s_rx_master_offset_entry, &c);

    for (int k = 0; k < 2; k++) dma_ch[k] = dma_claim_unused_channel(true);
    for (int k = 0; k < 2; k++) {
        dma_channel_config cfg = dma_channel_get_default_config(dma_ch[k]);
        channel_config_set_transfer_data_size(&cfg, DMA_SIZE_32);
        channel_config_set_read_increment(&cfg, false);
        channel_config_set_write_increment(&cfg, true);
        channel_config_set_dreq(&cfg, pio_get_dreq(I2S_PIO, I2S_SM, false));
        channel_config_set_chain_to(&cfg, dma_ch[k ^ 1]);
        dma_channel_configure(dma_ch[k], &cfg, bufs[k], &I2S_PIO->rxf[I2S_SM], BLOCK_WORDS, false);
        dma_channel_set_irq0_enabled(dma_ch[k], true);
    }
    irq_set_exclusive_handler(DMA_IRQ_0, dma_irq_handler);
    irq_set_priority(DMA_IRQ_0, PICO_HIGHEST_IRQ_PRIORITY);
    irq_set_enabled(DMA_IRQ_0, true);

    dma_channel_start(dma_ch[0]);
    pio_sm_set_enabled(I2S_PIO, I2S_SM, true);
}

bool i2s_block_take(uint32_t *seq, const int32_t **frames, uint64_t *t_us, bool *skipped) {
    uint32_t c = completed;
    *skipped = false;
    if (c == processed) return false;
    if (c - processed > 1) {            /* fell behind: only the newest complete block is intact */
        processed = c - 1;
        *skipped = true;
    }
    uint32_t j = processed;
    *seq = j + 1;
    *frames = bufs[j & 1];
    *t_us = t_done[j & 1];
    return true;
}

void i2s_block_release(void) {
    processed++;
}

uint32_t i2s_blocks_completed(void) { return completed; }
uint32_t i2s_overruns(void) { return overruns; }
uint32_t i2s_block_period_us(void) { return period_us; }

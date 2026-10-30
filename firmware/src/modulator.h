/*
 * modulator.h - LED square-wave modulator (PIO0 SM1 on GP0), armed at block
 * boundaries so that its phase origin is a known sample frame.
 */
#pragma once
#include <stdbool.h>
#include <stdint.h>

void modulator_init(void);
void modulator_request_start(uint32_t p);       /* (re)start with period p at the next block boundary */
void modulator_off(void);                       /* immediate, safe from any context */
bool modulator_running(void);                   /* state machine armed or toggling */
void modulator_block_boundary(uint32_t completed_blocks, int active_dma_ch);   /* DMA IRQ only */
/* Latest start: block seq it applies from, its period, the frame of that
 * block the LED rising edge sits in (-1 unknown) and a counter that changes
 * with every start. Returns false if the modulator never started. */
bool modulator_start_info(uint32_t *seq, uint32_t *p, int *frame, uint32_t *count);
const char *modulator_sync_str(void);
/* words of the start block the DMA had received when the block IRQ reached the
 * modulator and after arming, and how long arming took (diagnostics) */
void modulator_arm_debug(uint32_t *w_entry, uint32_t *w_armed, uint32_t *arm_us);

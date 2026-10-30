/*
 * i2s_rx.h - I2S master receiver (PIO0 SM0) with DMA ping-pong into two
 * 100 ms blocks of interleaved int32 LEFT/RIGHT words.
 */
#pragma once
#include <stdbool.h>
#include <stdint.h>

void i2s_init(void);
/* Next unprocessed block: seq (1-based), its words and the time its last
 * sample arrived. skipped = the main loop fell behind and older blocks were
 * dropped. Call i2s_block_release() when done with the buffer. */
bool i2s_block_take(uint32_t *seq, const int32_t **frames, uint64_t *t_us, bool *skipped);
void i2s_block_release(void);
uint32_t i2s_blocks_completed(void);
uint32_t i2s_overruns(void);
uint32_t i2s_block_period_us(void);     /* time between the last two block boundaries */

/* selftest.h - device self-test, reported as one JSON message. */
#pragma once
#include <stdint.h>

void selftest_init(void);               /* loads the frame counter on PIO1 */
void selftest_run(void);
uint32_t selftest_bclk_per_frame(void); /* measured on the running bus, 0 on timeout */

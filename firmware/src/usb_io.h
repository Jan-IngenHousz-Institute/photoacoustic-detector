/*
 * usb_io.h - USB CDC transport: non-blocking TX ring (messages are queued
 * whole or dropped), RX line assembly, DTR / 1200-baud / unmount hooks.
 */
#pragma once
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

typedef void (*usb_line_cb_t)(const char *line);

typedef enum {
    USB_MSG_CTRL,       /* id, status, ack, log, error, selftest, regs: kept while no host listens */
    USB_MSG_DATA,       /* lock-in result lines: dropped without a host */
    USB_MSG_BURST,      /* raw sample blocks: dropped without a host */
} usb_msg_kind_t;

void usb_init(usb_line_cb_t cb);
void usb_pump(void);                    /* run often from the main loop (never from an IRQ) */
bool usb_send(const char *data, size_t len, usb_msg_kind_t kind);
bool usb_connected(void);               /* DTR asserted by the host */
uint32_t usb_drop_burst(void);
uint32_t usb_drop_data(void);

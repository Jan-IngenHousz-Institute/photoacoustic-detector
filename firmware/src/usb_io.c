/*
 * usb_io.c - see usb_io.h. TinyUSB is driven only from the main loop
 * (usb_pump); the callbacks below therefore run in main-loop context too.
 */
#include "usb_io.h"
#include "app.h"
#include "modulator.h"

#include <string.h>
#include "pico/bootrom.h"
#include "pico/unique_id.h"
#include "tusb.h"

#define TX_RING     32768u              /* power of two */
#define TX_MASK     (TX_RING - 1)
#define RX_LINE_MAX 255

static uint8_t ring[TX_RING];
static uint32_t head, tail;             /* main loop only */
static uint32_t drop_burst, drop_data;
static char rx_line[RX_LINE_MAX + 1];
static size_t rx_len;
static bool rx_overflow;
static usb_line_cb_t line_cb;

void usb_init(usb_line_cb_t cb) {
    line_cb = cb;
    tud_init(0);
}

bool usb_connected(void) {
    return tud_cdc_connected();
}

static uint32_t ring_used(void) { return (head - tail) & TX_MASK; }

bool usb_send(const char *data, size_t len, usb_msg_kind_t kind) {
    bool is_burst = kind == USB_MSG_BURST;
    if (!tud_cdc_connected() && kind != USB_MSG_CTRL) return false;   /* nobody listening: keep logs only */
    if (len > TX_MASK - ring_used()) {
        if (is_burst) drop_burst++; else drop_data++;
        return false;
    }
    uint32_t first = TX_RING - (head & TX_MASK);
    if (first > len) first = (uint32_t)len;
    memcpy(ring + (head & TX_MASK), data, first);
    if (len > first) memcpy(ring, data + first, len - first);
    head = (head + (uint32_t)len) & TX_MASK;
    return true;
}

static void rx_feed(const uint8_t *b, uint32_t n) {
    for (uint32_t i = 0; i < n; i++) {
        char c = (char)b[i];
        if (c == '\n' || c == '\r') {
            if (rx_overflow) {
                rx_overflow = false;
            } else if (rx_len) {
                rx_line[rx_len] = 0;
                line_cb(rx_line);
            }
            rx_len = 0;
        } else if (rx_overflow) {
            continue;
        } else if (rx_len < RX_LINE_MAX) {
            rx_line[rx_len++] = c;
        } else {
            rx_overflow = true;         /* overlong line: discard it entirely */
            rx_len = 0;
        }
    }
}

void usb_pump(void) {
    tud_task();
    if (!tud_cdc_connected()) {
        drop_burst = drop_data = 0;
        return;
    }
    while (head != tail) {
        uint32_t avail = tud_cdc_write_available();
        if (!avail) break;
        uint32_t n = ring_used();
        if (n > avail) n = avail;
        uint32_t contiguous = TX_RING - tail;
        if (n > contiguous) n = contiguous;
        uint32_t w = tud_cdc_write(ring + tail, n);
        tail = (tail + w) & TX_MASK;
        if (w < n) break;
    }
    tud_cdc_write_flush();
    while (tud_cdc_available()) {
        uint8_t b[64];
        uint32_t n = tud_cdc_read(b, sizeof b);
        if (!n) break;
        rx_feed(b, n);
    }
}

uint32_t usb_drop_burst(void) { return drop_burst; }
uint32_t usb_drop_data(void) { return drop_data; }

/* ---- TinyUSB callbacks (main-loop context) --------------------------------- */

static void host_gone(void) {
    modulator_off();
    app.light = false;
    head = tail = 0;                    /* stale stream is useless to the next client */
    tud_cdc_write_clear();              /* and so is what the old host never read: the next one must
                                           not receive the tail of a burst glued to its first reply */
    tud_cdc_read_flush();
    rx_len = 0;
    rx_overflow = false;
}

void tud_cdc_line_state_cb(uint8_t itf, bool dtr, bool rts) {
    (void)itf; (void)rts;
    if (!dtr) {
        host_gone();
    } else {
        usb_send("\n", 1, USB_MSG_CTRL);  /* new host: terminate anything left unfinished */
    }
}

void tud_cdc_line_coding_cb(uint8_t itf, cdc_line_coding_t const *p) {
    (void)itf;
    if (p->bit_rate == 1200) {          /* the usual "touch" to reach the bootloader */
        modulator_off();
        reset_usb_boot(0, 0);
    }
}

void tud_umount_cb(void) { host_gone(); }
void tud_suspend_cb(bool remote_wakeup_en) { (void)remote_wakeup_en; host_gone(); }

/*
 * safety.c - see safety.h.
 *
 * The LED must never stay on without the PC asking for it: the heartbeat
 * timer switches the modulator off when no command arrived for heartbeat_s
 * seconds, independently of the main loop; the watchdog reboots a stuck
 * firmware (GP0 then floats until main() drives it low again: the board's
 * pull-down keeps the LED off); panic and hard faults drive GP0 low and reboot.
 */
#include "safety.h"
#include "app.h"
#include "modulator.h"

#include <stdarg.h>
#include "hardware/gpio.h"
#include "hardware/watchdog.h"
#include "pico/time.h"

static repeating_timer_t hb_timer;

static bool heartbeat_cb(repeating_timer_t *rt) {
    (void)rt;
    if (app.light && app.heartbeat_s) {
        uint64_t now = time_us_64();
        if (now - app.last_cmd_us > (uint64_t)app.heartbeat_s * 1000000ull) {
            modulator_off();
            app.light = false;
            app.heartbeat_lost = true;
        }
    }
    return true;
}

void safety_init(void) {
    watchdog_enable(WATCHDOG_MS, true);
    add_repeating_timer_ms(50, heartbeat_cb, NULL, &hb_timer);
}

static void __attribute__((noreturn)) die(void) {
    gpio_set_function(PIN_MOD, GPIO_FUNC_SIO);
    gpio_set_dir(PIN_MOD, true);
    gpio_put(PIN_MOD, 0);
    watchdog_reboot(0, 0, 10);
    while (1) __asm volatile("wfi");
}

void __attribute__((noreturn)) pa_panic(const char *fmt, ...) {
    (void)fmt;
    die();
}

void isr_hardfault(void) {
    die();
}

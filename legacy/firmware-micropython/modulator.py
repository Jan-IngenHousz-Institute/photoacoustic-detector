"""
modulator.py - PIO square-wave modulation of the LED current source, locked to
the I2S bit clock.

A PIO state machine counts BCLK rising edges (GP6, 64 per sample frame) and
drives the current source control line high for H bit clocks and low for L bit
clocks. The period P = H + L is a whole number of bit clocks, so
f_mod / fs = 64 / P exactly and the lock-in reference (lockin.py) stays phase
coherent with the light for as long as both run:

    f_mod = 64 * fs / P        P = 308 .. 15360  ->  9974 Hz .. 200 Hz at 48 kHz

MicroPython's rp2 I2S receiver clocks 32-bit slots even for 16-bit samples, so
a frame is 64 bit clocks, not 32; measure_bclk_per_frame() checks this on the
running bus (self-test).

Config word pushed into the TX FIFO (picked up at the start of the next period):
    bits 15..0  : high time in bit clocks - 1
    bits 31..16 : low time in bit clocks - 1
"""

import time

import rp2
from machine import Pin
from micropython import const

BCLK_GPIO = const(6)      # I2S sck / ws pins watched by the programs (absolute GPIO);
FSYNC_GPIO = const(7)     # const() so the values are inlined, asm_pio hides module globals
BCLK_PER_FRAME = 64       # 2 x 32-bit slots per I2S frame


@rp2.asm_pio(set_init=rp2.PIO.OUT_LOW, out_shiftdir=rp2.PIO.SHIFT_RIGHT)
def _square_prog():
    wrap_target()
    pull(noblock)                  # new config, or X (previous config) if FIFO empty
    mov(x, osr)
    out(y, 16)                     # Y = high bit clocks - 1
    set(pins, 1)
    label("high")
    wait(0, gpio, BCLK_GPIO)
    wait(1, gpio, BCLK_GPIO)       # one BCLK rising edge
    jmp(y_dec, "high")
    out(y, 16)                     # Y = low bit clocks - 1
    set(pins, 0)
    label("low")
    wait(0, gpio, BCLK_GPIO)
    wait(1, gpio, BCLK_GPIO)
    jmp(y_dec, "low")
    wrap()


@rp2.asm_pio()
def _slot_bits_prog():
    wait(0, gpio, FSYNC_GPIO)
    wait(1, gpio, FSYNC_GPIO)      # start of the right-channel half frame
    mov(x, invert(null))
    label("count")
    wait(0, gpio, BCLK_GPIO)
    wait(1, gpio, BCLK_GPIO)
    jmp(x_dec, "next")             # always falls through: just X -= 1
    label("next")
    jmp(pin, "count")              # FSYNC still high
    mov(isr, invert(x))            # bit clocks counted
    push()


def measure_bclk_per_frame(sm_id=5, timeout_ms=50):
    """Bit clocks per I2S frame on the running bus (2 x those while FSYNC is high)."""
    sm = rp2.StateMachine(sm_id, _slot_bits_prog, jmp_pin=Pin(FSYNC_GPIO))
    sm.active(1)
    t0 = time.ticks_ms()
    while not sm.rx_fifo() and time.ticks_diff(time.ticks_ms(), t0) < timeout_ms:
        pass
    n = 2 * sm.get() if sm.rx_fifo() else 0
    sm.active(0)
    return n


class Modulator:
    def __init__(self, pin_no, period=BCLK_PER_FRAME * 48, sm_id=4):
        self.pin_no = pin_no
        self.period = int(period)
        self.active = False
        self.pin = Pin(pin_no, Pin.OUT, value=0)
        # SM 4..7 live on PIO1, away from the I2S SM; full system clock so no
        # BCLK edge (1.536 MHz) is missed
        self.sm = rp2.StateMachine(sm_id, _square_prog, set_base=self.pin)

    def _word(self):
        high = self.period // 2              # 50 % duty, +-1 bit clock for odd P
        low = self.period - high
        return ((low - 1) << 16) | (high - 1)

    def configure(self, period):
        self.period = int(period)
        if self.active and self.sm.tx_fifo() < 4:
            self.sm.put(self._word())        # takes effect at the next period

    def start(self):
        self.sm.active(0)
        self.sm.restart()
        self.sm.put(self._word())
        self.sm.active(1)
        self.active = True

    def stop(self):
        self.sm.active(0)
        self.sm.exec("set(pins, 0)")         # never leave the LED current on
        self.active = False

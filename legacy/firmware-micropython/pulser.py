"""
pulser.py - PIO pulse generator phase-locked to the I2S frame clock.

A PIO state machine watches the I2S word-select line (FSYNC, GP7), counts N
frames (= N ADC samples) and then drives the actopulser PULSE line high for a
programmable time. Because the pulse always lands on the same sample index of
every period, the lock-in phase measured on the PC is stable and long coherent
averaging is possible.

Config word pushed into the TX FIFO (picked up at the next period):
    bits 15..0  : frames per period - 1
    bits 31..16 : pulse width in PIO cycles - 2   (PIO clock 2 MHz -> 0.5 us)

Keep the pulse width below one frame period (20.8 us at 48 kHz) so no FSYNC
edges are missed while the pulse is high.
"""

import rp2
from machine import Pin

FSYNC_GPIO = 7            # I2S ws pin watched by the program (absolute GPIO)
PIO_FREQ = 2_000_000      # 0.5 us per PIO cycle
_MIN_CYCLES = 2           # shortest high time the loop can produce (1 us)


@rp2.asm_pio(set_init=rp2.PIO.OUT_LOW, out_shiftdir=rp2.PIO.SHIFT_RIGHT)
def _pulser_prog():
    wrap_target()
    pull(noblock)                  # new config, or X (previous config) if FIFO empty
    mov(x, osr)
    out(y, 16)                     # Y = frames per period - 1
    label("count")
    wait(0, gpio, FSYNC_GPIO)
    wait(1, gpio, FSYNC_GPIO)      # one FSYNC rising edge = one sample frame
    jmp(y_dec, "count")
    out(y, 16)                     # Y = width cycles - 2
    set(pins, 1)
    label("high")
    jmp(y_dec, "high")
    set(pins, 0)
    wrap()


class Pulser:
    def __init__(self, pin_no, period_frames=48, width_us=10.0, sm_id=4):
        self.sm_id = sm_id             # SM 4..7 live on PIO1, away from the I2S SM
        self.pin_no = pin_no
        self.period = int(period_frames)
        self.width_us = float(width_us)
        self.active = False
        self._make()

    def _make(self):
        self.pin = Pin(self.pin_no, Pin.OUT, value=0)
        self.sm = rp2.StateMachine(self.sm_id, _pulser_prog, freq=PIO_FREQ,
                                   set_base=self.pin)

    def _word(self):
        frames = max(1, self.period) - 1
        cycles = int(round(self.width_us * PIO_FREQ / 1_000_000))
        cycles = max(_MIN_CYCLES, cycles) - _MIN_CYCLES
        return ((cycles & 0xFFFF) << 16) | (frames & 0xFFFF)

    def actual_width_us(self):
        cycles = max(_MIN_CYCLES, int(round(self.width_us * PIO_FREQ / 1_000_000)))
        return cycles * 1_000_000 / PIO_FREQ

    def configure(self, period_frames=None, width_us=None):
        if period_frames is not None:
            self.period = int(period_frames)
        if width_us is not None:
            self.width_us = float(width_us)
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
        self.pin.value(0)
        self.active = False

    def set_pin(self, pin_no):
        """Move the pulse output to another actopulser channel line."""
        was = self.active
        self.stop()
        Pin(self.pin_no, Pin.IN)             # release the old line (100k pull-down on board)
        self.pin_no = pin_no
        self._make()
        if was:
            self.start()

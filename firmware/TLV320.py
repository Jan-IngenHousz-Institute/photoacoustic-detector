"""
TLV320.py - MicroPython driver for the TI TLV320ADC6120 audio ADC.

Control is over I2C (fixed 7-bit address 0x4C); audio data comes out over
I2S/TDM on SDOUT. On the photoacoustic-detector shield the ADC is wired to
the RP2040 Pico as follows (from kicad/photoacoustic-detector.kicad_pcb):

    ADC pin            net     Pico pin
    SDA  (12)          SDA     GP4   (I2C0 SDA, 2k2 pull-up to 3V3)
    SCL  (13)          SCL     GP5   (I2C0 SCL, 2k2 pull-up to 3V3)
    BCLK (7)           BCLK    GP6   (I2S bit clock, Pico is master)
    FSYNC (8)          FSYNC   GP7   (I2S word select, Pico is master)
    SDOUT (6)          SDOUT   GP8   (I2S data, ADC -> Pico)
    GPIO1 (11)         EXT     GP9   (default: ADC interrupt output)
    IN1P/IN1M (1,2)    OUT+/-  AC-coupled differential microphone (IM73A135)
    MICBIAS (19)               microphone supply (default VREF = 2.75 V)
    AVDD, IOVDD                3V3 -> internal 1.8 V AREG regulator is used

Minimal use:

    from machine import I2C, Pin
    import TLV320
    i2c = I2C(0, sda=Pin(4), scl=Pin(5), freq=400_000)
    adc = TLV320.TLV320ADC6120(i2c)
    if adc.detect():
        adc.begin(bits=32, gain_db=12)      # reset, wake, configure, power up
        ...create machine.I2S RX at 48 kHz, 32 bit, stereo; CH1 is the LEFT slot
        print(adc.status())

Register map reference: TI SBASA92A, section 8.6.2 (page 0).
"""

from micropython import const
import time

I2C_ADDR = const(0x4C)

# ---- Page 0 register addresses -------------------------------------------
PAGE_CFG      = const(0x00)
SW_RESET      = const(0x01)
SLEEP_CFG     = const(0x02)
SHDN_CFG      = const(0x05)
ASI_CFG0      = const(0x07)
ASI_CFG1      = const(0x08)
ASI_CFG2      = const(0x09)
ASI_CH1       = const(0x0B)
ASI_CH2       = const(0x0C)
MST_CFG0      = const(0x13)
MST_CFG1      = const(0x14)
ASI_STS       = const(0x15)
CLK_SRC       = const(0x16)
GPIO_CFG0     = const(0x21)
GPO_CFG0      = const(0x22)
GPO_VAL       = const(0x29)
GPIO_MON      = const(0x2A)
INT_CFG       = const(0x32)
INT_MASK0     = const(0x33)
INT_LTCH0     = const(0x36)
BIAS_CFG      = const(0x3B)
CH1_CFG0      = const(0x3C)
CH1_CFG1      = const(0x3D)
CH1_CFG2      = const(0x3E)
CH1_CFG3      = const(0x3F)
CH1_CFG4      = const(0x40)
CH2_CFG0      = const(0x41)
CH2_CFG1      = const(0x42)
CH2_CFG2      = const(0x43)
CH2_CFG3      = const(0x44)
CH2_CFG4      = const(0x45)
DSP_CFG0      = const(0x6B)
DSP_CFG1      = const(0x6C)
DRE_CFG0      = const(0x6D)
AGC_CFG0      = const(0x70)
GAIN_CFG      = const(0x71)
IN_CH_EN      = const(0x73)
ASI_OUT_CH_EN = const(0x74)
PWR_CFG       = const(0x75)
DEV_STS0      = const(0x76)
DEV_STS1      = const(0x77)

# ---- Field encodings ------------------------------------------------------
FMT_TDM = const(0)
FMT_I2S = const(1)
FMT_LJ  = const(2)

_WLEN = {16: 0, 20: 1, 24: 2, 32: 3}
_IMP = {2500: 0, 10000: 1, 20000: 2}

# ASI_STS decode tables
_FS_RATE = {0: "8k", 1: "16k", 2: "24k", 3: "32k", 4: "48k", 5: "96k",
            6: "192k", 7: "384k", 8: "768k"}
_FS_RATIO = {0: 16, 1: 24, 2: 32, 3: 48, 4: 64, 5: 96, 6: 128, 7: 192,
             8: 256, 9: 384, 10: 512, 11: 1024, 12: 2048}
_MODE_STS = {4: "sleep/shutdown", 6: "active, channels off",
             7: "active, channel(s) on"}

# Page 0 defaults after reset - used by verify() for a sanity check.
_RESET_DEFAULTS = (
    (SLEEP_CFG, 0x00), (SHDN_CFG, 0x05), (ASI_CFG0, 0x30), (MST_CFG1, 0x48),
    (CLK_SRC, 0x10), (GPIO_CFG0, 0x22), (CH1_CFG2, 0xC9), (CH1_CFG3, 0x80),
    (CH2_CFG2, 0xC9), (DSP_CFG0, 0x01), (DSP_CFG1, 0x40), (IN_CH_EN, 0xC0),
    (DEV_STS1, 0x80),
)


class TLV320ADC6120:
    def __init__(self, i2c, addr=I2C_ADDR):
        self.i2c = i2c
        self.addr = addr
        self._buf1 = bytearray(1)

    # ---- low level -------------------------------------------------------
    def write_reg(self, reg, val):
        self.i2c.writeto_mem(self.addr, reg, bytes((val & 0xFF,)))

    def read_reg(self, reg):
        self.i2c.readfrom_mem_into(self.addr, reg, self._buf1)
        return self._buf1[0]

    def read_regs(self, reg, n):
        return self.i2c.readfrom_mem(self.addr, reg, n)

    def update_reg(self, reg, mask, val):
        """Read-modify-write: bits in `mask` are replaced by `val`."""
        cur = self.read_reg(reg)
        new = (cur & ~mask) | (val & mask)
        if new != cur:
            self.write_reg(reg, new)
        return new

    def set_page(self, page):
        self.write_reg(PAGE_CFG, page)

    # ---- presence / identification ---------------------------------------
    def present(self):
        """True if something ACKs at the ADC's I2C address."""
        return self.addr in self.i2c.scan()

    def detect(self):
        """
        Presence check that also proves a register-backed device is there:
        the device must ACK, and PAGE_CFG must read back what was written.
        The device has no ID register, so this is the strongest safe test.
        Non-destructive (page is restored to 0).
        """
        if not self.present():
            return False
        try:
            self.set_page(1)
            ok = self.read_reg(PAGE_CFG) == 1
            self.set_page(0)
            ok = ok and self.read_reg(PAGE_CFG) == 0
            return ok
        except OSError:
            return False

    def verify(self):
        """
        Call right after reset(): compares a set of page-0 registers against
        their datasheet reset values. Returns list of (reg, got, expected)
        mismatches; empty list means the part behaves like a TLV320ADC6120.
        """
        bad = []
        for reg, exp in _RESET_DEFAULTS:
            got = self.read_reg(reg)
            if got != exp:
                bad.append((reg, got, exp))
        return bad

    # ---- power state -----------------------------------------------------
    def reset(self):
        """Software reset. Device ends up in sleep mode with default registers."""
        self.set_page(0)
        self.write_reg(SW_RESET, 0x01)
        time.sleep_ms(2)           # datasheet: >= 1 ms

    def wake(self, internal_areg=True):
        """
        Leave sleep mode. internal_areg=True selects the on-chip 1.8 V AREG
        regulator, required when AVDD = 3.3 V (this board).
        """
        val = 0x01 | (0x80 if internal_areg else 0x00)
        self.write_reg(SLEEP_CFG, val)
        time.sleep_ms(5)           # >= 1 ms wake-up, VREF quick-charge ~3.5 ms

    def sleep(self):
        """Enter sleep mode (registers retained). Stop I2S clocks afterwards."""
        self.update_reg(SLEEP_CFG, 0x01, 0x00)
        time.sleep_ms(10)

    # ---- configuration ---------------------------------------------------
    def config_asi(self, fmt=FMT_I2S, bits=32, fsync_inv=False, bclk_inv=False):
        """Audio serial interface format. Slave mode (BCLK/FSYNC are inputs)."""
        if bits not in _WLEN:
            raise ValueError("bits must be 16, 20, 24 or 32")
        val = (fmt << 6) | (_WLEN[bits] << 4)
        if fsync_inv:
            val |= 0x08
        if bclk_inv:
            val |= 0x04
        self.write_reg(ASI_CFG0, val)
        # Slave mode, auto clock config with PLL (defaults) - make explicit.
        self.write_reg(MST_CFG0, 0x00)

    def config_channel(self, ch, line_in=False, single_ended=False,
                       dc_coupled=False, impedance=2500, dre=False):
        """Analog front-end for channel 1 or 2."""
        base = CH1_CFG0 if ch == 1 else CH2_CFG0
        if impedance not in _IMP:
            raise ValueError("impedance must be 2500, 10000 or 20000")
        val = (_IMP[impedance] << 2)
        if line_in:
            val |= 0x80
        if single_ended:
            val |= 0x20
        if dc_coupled:
            val |= 0x10
        if dre:
            val |= 0x01
        self.write_reg(base, val)

    def set_gain(self, ch, db):
        """Analog channel gain, 0 .. 42 dB in 0.5 dB steps."""
        code = int(round(db * 2))
        if not 0 <= code <= 84:
            raise ValueError("gain 0..42 dB")
        reg = CH1_CFG1 if ch == 1 else CH2_CFG1
        self.write_reg(reg, code << 1)

    def set_volume(self, ch, db=0, mute=False):
        """Digital volume, -100 .. +27 dB in 0.5 dB steps (0 dB default)."""
        if mute:
            code = 0
        else:
            code = 201 + int(round(db * 2))
            if not 1 <= code <= 255:
                raise ValueError("volume -100..27 dB")
        reg = CH1_CFG2 if ch == 1 else CH2_CFG2
        self.write_reg(reg, code)

    def set_hpf(self, sel=1):
        """HPF: 0 = custom IIR (page coefficients), 1 = 12 Hz, 2 = 96 Hz,
        3 = 384 Hz (cut-offs at fs = 48 kHz)."""
        self.update_reg(DSP_CFG0, 0x03, sel & 0x03)

    def set_decimation(self, mode=0):
        """0 = linear phase, 1 = low latency, 2 = ultra-low latency."""
        self.update_reg(DSP_CFG0, 0x30, (mode & 0x03) << 4)

    def set_micbias(self, mbias_val=0, fscale=0):
        """
        mbias_val: 0 = VREF (2.75 V default), 1 = VREF x 1.096, 6 = AVDD.
        fscale:    0 = VREF 2.75 V (2 Vrms diff), 1 = 2.5 V, 2 = 1.375 V.
        """
        self.write_reg(BIAS_CFG, ((mbias_val & 0x07) << 4) | (fscale & 0x03))

    def enable_channels(self, ch1=True, ch2=False):
        """Enable ADC input channels and their ASI output slots."""
        val = (0x80 if ch1 else 0) | (0x40 if ch2 else 0)
        self.write_reg(IN_CH_EN, val)
        self.write_reg(ASI_OUT_CH_EN, val)

    def power_up(self, micbias=True, adc=True, pll=True):
        val = (0x80 if micbias else 0) | (0x40 if adc else 0) | (0x20 if pll else 0)
        self.write_reg(PWR_CFG, val)

    def power_down(self):
        self.write_reg(PWR_CFG, 0x00)

    def begin(self, bits=32, fmt=FMT_I2S, gain_db=0, ch2=False, hpf=1,
              line_in=False, micbias=True):
        """
        Full bring-up for this board: reset -> wake (internal AREG) ->
        I2S slave -> CH1 differential mic input -> enable -> power up.
        After this, start the I2S master clocks on the Pico; the ADC locks
        its PLL to BCLK/FSYNC automatically (48 kHz, ratio 32 or 64 OK).
        """
        self.reset()
        self.wake(internal_areg=True)
        self.config_asi(fmt=fmt, bits=bits)
        self.config_channel(1, line_in=line_in)
        self.set_gain(1, gain_db)
        self.set_volume(1, 0)
        if ch2:
            self.config_channel(2, line_in=line_in)
            self.set_gain(2, gain_db)
            self.set_volume(2, 0)
        self.set_hpf(hpf)
        self.enable_channels(ch1=True, ch2=ch2)
        self.power_up(micbias=micbias, adc=True, pll=True)

    # ---- status ----------------------------------------------------------
    def status(self):
        """Decoded status dictionary (mode, channel power, detected clocks)."""
        sts0 = self.read_reg(DEV_STS0)
        sts1 = self.read_reg(DEV_STS1)
        asi = self.read_reg(ASI_STS)
        mode = sts1 >> 5
        rate = asi >> 4
        ratio = asi & 0x0F
        return {
            "mode": _MODE_STS.get(mode, "0x%X" % mode),
            "ch1_on": bool(sts0 & 0x80),
            "ch2_on": bool(sts0 & 0x40),
            "fs": _FS_RATE.get(rate, "invalid"),
            "bclk_ratio": _FS_RATIO.get(ratio, "invalid"),
            "clock_ok": rate != 0xF and ratio != 0xF,
            "int_latch": self.read_reg(INT_LTCH0),
        }

    def dump(self):
        """Print all documented page-0 registers."""
        self.set_page(0)
        regs = (PAGE_CFG, SW_RESET, SLEEP_CFG, SHDN_CFG, ASI_CFG0, ASI_CFG1,
                ASI_CFG2, 0x0A, ASI_CH1, ASI_CH2, 0x0D, 0x0E, MST_CFG0,
                MST_CFG1, ASI_STS, CLK_SRC, 0x1F, 0x20, GPIO_CFG0, GPO_CFG0,
                GPO_VAL, GPIO_MON, 0x2B, 0x2F, INT_CFG, INT_MASK0, INT_LTCH0,
                0x3A, BIAS_CFG, CH1_CFG0, CH1_CFG1, CH1_CFG2, CH1_CFG3,
                CH1_CFG4, CH2_CFG0, CH2_CFG1, CH2_CFG2, CH2_CFG3, CH2_CFG4,
                DSP_CFG0, DSP_CFG1, DRE_CFG0, AGC_CFG0, GAIN_CFG, IN_CH_EN,
                ASI_OUT_CH_EN, PWR_CFG, DEV_STS0, DEV_STS1)
        for r in regs:
            print("P0_R%-3d 0x%02X = 0x%02X" % (r, r, self.read_reg(r)))

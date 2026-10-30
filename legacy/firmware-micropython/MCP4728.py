"""
MCP4728.py - MicroPython driver for the Microchip MCP4728 quad 12-bit I2C DAC,
as used on the actoPulserX4-48V current source.

Actopulser channel: DAC VoutX -> analog switch (gated by PULSEx) -> op-amp ->
MOSFET current sink with a 2 ohm sense resistor, so

    I_LED = Vout / 2 ohm

With the internal 2.048 V reference and gain x2 the DAC spans 0..4.095 V,
i.e. 0..2047.5 mA in 0.5 mA steps, independent of the board's 5 V regulator.

The actopulser has no I2C pull-ups of its own: enable the Pico's internal
pull-ups on SDA/SCL (or fit external 4k7 to 3V3).
"""

from micropython import const

I2C_ADDR = const(0x60)          # A2..A0 = 000 (factory default)

VREF_VDD = const(0)
VREF_INT = const(1)             # 2.048 V internal reference
GAIN_1 = const(0)
GAIN_2 = const(1)
PD_NORMAL = const(0)
PD_1K = const(1)
PD_100K = const(2)
PD_500K = const(3)

MA_PER_LSB = 0.5                # with VREF_INT + GAIN_2 into 2 ohm


class MCP4728:
    def __init__(self, i2c, addr=I2C_ADDR, vref=VREF_INT, gain=GAIN_2):
        self.i2c = i2c
        self.addr = addr
        self.vref = vref
        self.gain = gain

    # ---- presence -----------------------------------------------------------
    def present(self):
        return self.addr in self.i2c.scan()

    # ---- writes (input register + output, EEPROM untouched) -----------------
    def _frame(self, ch, code, pd):
        code = 0 if code < 0 else (4095 if code > 4095 else int(code))
        b0 = 0x40 | ((ch & 3) << 1)              # multi-write, UDAC = 0
        b1 = (self.vref << 7) | ((pd & 3) << 5) | (self.gain << 4) | (code >> 8)
        return bytes((b0, b1, code & 0xFF))

    def write(self, ch, code, pd=PD_NORMAL):
        """Set one channel (0..3) to a 12-bit code; output updates at once."""
        self.i2c.writeto(self.addr, self._frame(ch, code, pd))

    def write_all(self, codes, pd=PD_NORMAL):
        """Set all four channels in one transaction."""
        buf = b"".join(self._frame(ch, codes[ch], pd) for ch in range(4))
        self.i2c.writeto(self.addr, buf)

    def off_all(self):
        self.write_all((0, 0, 0, 0))

    # ---- readback -------------------------------------------------------------
    def read(self):
        """
        Read back all channels. Returns a list of 4 dicts with the live DAC
        register (code, vref, gain, pd) and the EEPROM copy (eeprom_code).
        A failed read raises OSError (device absent).
        """
        d = self.i2c.readfrom(self.addr, 24)
        out = []
        for ch in range(4):
            r = d[6 * ch: 6 * ch + 3]
            e = d[6 * ch + 3: 6 * ch + 6]
            out.append({
                "ch": ch,
                "code": ((r[1] & 0x0F) << 8) | r[2],
                "vref": (r[1] >> 7) & 1,
                "pd": (r[1] >> 5) & 3,
                "gain": (r[1] >> 4) & 1,
                "eeprom_code": ((e[1] & 0x0F) << 8) | e[2],
            })
        return out

    # ---- conversions ----------------------------------------------------------
    def full_scale_volts(self, vdd=5.0):
        if self.vref == VREF_INT:
            return 2.048 * (2 if self.gain == GAIN_2 else 1)
        return vdd

    def code_to_volts(self, code, vdd=5.0):
        return self.full_scale_volts(vdd) * code / 4096.0

    @staticmethod
    def ma_to_code(ma):
        """LED current in mA -> DAC code (VREF_INT, GAIN_2, 2 ohm sense)."""
        code = int(round(ma / MA_PER_LSB))
        return 0 if code < 0 else (4095 if code > 4095 else code)

    @staticmethod
    def code_to_ma(code):
        return code * MA_PER_LSB

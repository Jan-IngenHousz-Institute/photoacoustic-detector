"""
lockin.py - block lock-in (single-bin DFT) for the TLV320 sample stream.

The I2S buffer holds interleaved 16-bit stereo frames; channel 1 of the ADC is
the LEFT slot (even indices). For a block whose length is a whole number of
modulation periods, the in-phase/quadrature sums against a one-period
sine/cosine table give the amplitude and phase at the modulation frequency
with no spectral leakage.

Integer arithmetic in a viper function (32-bit machine ints): every product
sample (16 bit) x table (14 bit) is split into a high part (>> 16, arithmetic
shift) and a low part (& 0xFFFF) that are accumulated separately, so the sums
over 4800 samples are exact and recombined in Python big ints.
"""

import math
from array import array
import micropython

TABLE_SCALE = 16383        # sine/cosine table amplitude


@micropython.viper
def _mac(buf: ptr16, nframes: int, ts: ptr16, tc: ptr16, period: int, out: ptr32):
    i_hi = 0
    i_lo = 0
    q_hi = 0
    q_lo = 0
    s_hi = 0
    s_lo = 0
    peak = 0
    k = 0
    for n in range(nframes):
        v = int(buf[n * 2])            # left slot = ADC channel 1
        if v >= 32768:
            v -= 65536
        s = int(ts[k])
        if s >= 32768:
            s -= 65536
        c = int(tc[k])
        if c >= 32768:
            c -= 65536
        p = v * c
        i_hi += p >> 16
        i_lo += p & 0xFFFF
        p = v * s
        q_hi += p >> 16
        q_lo += p & 0xFFFF
        p = v * v
        s_hi += p >> 16
        s_lo += p & 0xFFFF
        a = v
        if a < 0:
            a = -a
        if a > peak:
            peak = a
        k += 1
        if k == period:
            k = 0
    out[0] = i_hi
    out[1] = i_lo
    out[2] = q_hi
    out[3] = q_lo
    out[4] = s_hi
    out[5] = s_lo
    out[6] = peak


@micropython.viper
def deinterleave_left(src: ptr16, dst: ptr16, n: int):
    """Copy the left (channel 1) samples of n stereo frames into dst."""
    for i in range(n):
        dst[i] = src[i * 2]


class LockIn:
    def __init__(self, period):
        self.out = array("i", [0] * 7)
        self.set_period(period)

    def set_period(self, n):
        """n = samples per modulation period (fs / f_mod), must divide the block."""
        n = int(n)
        self.period = n
        self.ts = array("h", (int(TABLE_SCALE * math.sin(2 * math.pi * k / n)) for k in range(n)))
        self.tc = array("h", (int(TABLE_SCALE * math.cos(2 * math.pi * k / n)) for k in range(n)))

    def process(self, buf, nframes):
        """
        buf: bytearray/memoryview of nframes interleaved int16 stereo frames.
        Returns (i, q, amp_fs, phase_deg, rms_fs, peak) where amp_fs is the
        sine amplitude as a fraction of full scale (1.0 = 0 dBFS peak) and
        rms_fs the broadband RMS as a fraction of full scale.
        """
        _mac(buf, nframes, self.ts, self.tc, self.period, self.out)
        o = self.out
        i = o[0] * 65536 + o[1]
        q = o[2] * 65536 + o[3]
        ss = o[4] * 65536 + o[5]
        peak = o[6]
        scale = 2.0 / (TABLE_SCALE * nframes * 32768.0)
        amp = math.sqrt(i * i + q * q) * scale
        ph = math.degrees(math.atan2(q, i))
        rms = math.sqrt(ss / nframes) / 32768.0
        return i, q, amp, ph, rms, peak

    @staticmethod
    def dbfs(x):
        return 20.0 * math.log10(x) if x > 0 else -200.0

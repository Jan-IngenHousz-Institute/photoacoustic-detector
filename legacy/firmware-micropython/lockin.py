"""
lockin.py - block lock-in (single-bin DFT) for the TLV320 sample stream.

The I2S buffer holds interleaved 16-bit stereo frames; channel 1 of the ADC is
the LEFT slot (even indices). The modulation period is P bit clocks
(modulator.py), i.e. P / 64 samples, which in general is not an integer and
does not divide the block. The reference is therefore a phase-continuous
oscillator: a 1024-entry sine/cosine table stepped by 64 * 1024 / P entries per
sample, with the fractional part carried Bresenham-style (no division, phase
error below 1/1024 of a cycle). Its phase carries over from block to block, so
signal and reference stay locked and the block results average coherently.

Integer arithmetic in a viper function (32-bit machine ints): every product
sample (16 bit) x table (14 bit) is split into a high part (>> 16, arithmetic
shift) and a low part (& 0xFFFF) that are accumulated separately, so the sums
over 4800 samples are exact and recombined in Python big ints.
"""

import math
from array import array
import micropython

TABLE_SCALE = 16383        # sine/cosine table amplitude
TABLE_LEN = 1024           # entries per reference cycle (the viper loop masks with 1023)


@micropython.viper
def _mac(buf: ptr16, nframes: int, ts: ptr16, tc: ptr16, st: ptr32, out: ptr32):
    idx = st[0]                        # table index of the reference phase
    rem = st[1]                        # fractional phase, in 1/p table entries
    q = st[2]
    r = st[3]
    p = st[4]
    i_hi = 0
    i_lo = 0
    q_hi = 0
    q_lo = 0
    s_hi = 0
    s_lo = 0
    peak = 0
    for n in range(nframes):
        v = int(buf[n * 2])            # left slot = ADC channel 1
        if v >= 32768:
            v -= 65536
        s = int(ts[idx])
        if s >= 32768:
            s -= 65536
        c = int(tc[idx])
        if c >= 32768:
            c -= 65536
        pr = v * c
        i_hi += pr >> 16
        i_lo += pr & 0xFFFF
        pr = v * s
        q_hi += pr >> 16
        q_lo += pr & 0xFFFF
        pr = v * v
        s_hi += pr >> 16
        s_lo += pr & 0xFFFF
        a = v
        if a < 0:
            a = -a
        if a > peak:
            peak = a
        idx += q
        rem += r
        if rem >= p:
            rem -= p
            idx += 1
        idx &= 1023
    st[0] = idx
    st[1] = rem
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
    def __init__(self, period_bclk, bclk_per_frame=64):
        self.out = array("i", [0] * 7)
        self.st = array("i", [0] * 5)           # idx, rem, q, r, p
        self.step = bclk_per_frame * TABLE_LEN  # table entries per sample = step / p
        self.ts = array("h", (int(TABLE_SCALE * math.sin(2 * math.pi * k / TABLE_LEN))
                              for k in range(TABLE_LEN)))
        self.tc = array("h", (int(TABLE_SCALE * math.cos(2 * math.pi * k / TABLE_LEN))
                              for k in range(TABLE_LEN)))
        self.ph0 = 0.0
        self.set_period(period_bclk)

    def set_period(self, p):
        """p = modulation period in I2S bit clocks (modulator.Modulator.period)."""
        p = int(p)
        self.period = p
        st = self.st
        st[1] = 0
        st[2] = self.step // p
        st[3] = self.step % p
        st[4] = p

    def phase(self):
        """Reference phase (cycles, 0..1) at the next sample to be processed."""
        st = self.st
        return (st[0] + st[1] / st[4]) / TABLE_LEN

    def process(self, buf, nframes):
        """
        buf: bytearray/memoryview of nframes interleaved int16 stereo frames.
        Returns (i, q, amp_fs, phase_deg, rms_fs, peak) where amp_fs is the
        sine amplitude as a fraction of full scale (1.0 = 0 dBFS peak) and
        rms_fs the broadband RMS as a fraction of full scale. self.ph0 is the
        reference phase at the first sample of this block.
        """
        self.ph0 = self.phase()
        _mac(buf, nframes, self.ts, self.tc, self.st, self.out)
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

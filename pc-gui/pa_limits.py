"""
pa_limits.py - modulation frequencies and amplitude calibration for the
photoacoustic detector GUI.

The LED (Ushio SMBB1900D-1100-02, 1 A continuous) is driven by an on/off
voltage-controlled current source on GP0. Its current (<= 1 A) is set in the
current source hardware and the firmware modulates it with a 50 % square wave,
so there are no software current or duty limits.

The modulation period is a whole number P of I2S bit clocks (64 per sample
frame: two 32-bit slots, BCLK = 3.072 MHz): f_mod = 64 * fs / P.
snap_frequency() mirrors the firmware rounding.

Sources:
  TLV320ADC6120: 2 Vrms differential full scale at 0 dB gain (VREF 2.75 V).
  IM73A135 microphone: -38 dBV/Pa sensitivity.
"""

import math

FS = 48000
BLOCK = 4800
BCLK_PER_FRAME = 64
F_MIN = 200.0
F_MAX = 10000.0
P_MIN = -(-BCLK_PER_FRAME * FS // int(F_MAX))   # 308 bit clocks = 9974 Hz (ceil: stay <= F_MAX)
P_MAX = BCLK_PER_FRAME * FS // int(F_MIN)       # 15360 bit clocks = 200 Hz

# ---- calibration defaults (editable in the GUI) -----------------------------
ADC_FULL_SCALE_VRMS = 2.0          # differential, 0 dB gain
MIC_SENS_DBV_PA = -38.0


def snap_frequency(freq_hz):
    """(P, f_actual): the modulation frequency the firmware makes for a request."""
    p = max(P_MIN, min(P_MAX, round(BCLK_PER_FRAME * FS / freq_hz)))
    return p, BCLK_PER_FRAME * FS / p


def sweep_frequencies(f0, f1, n, log=True):
    """n points from f0 to f1, log or linear spaced, snapped and de-duplicated."""
    lo, hi = sorted((f0, f1))
    if n < 2:
        targets = [lo]
    elif log:
        targets = [lo * (hi / lo) ** (k / (n - 1)) for k in range(n)]
    else:
        targets = [lo + (hi - lo) * k / (n - 1) for k in range(n)]
    return sorted({snap_frequency(f)[1] for f in targets})


class Calibration:
    def __init__(self, adc_fs_vrms=ADC_FULL_SCALE_VRMS, mic_sens_dbv_pa=MIC_SENS_DBV_PA):
        self.adc_fs_vrms = adc_fs_vrms
        self.mic_sens_dbv_pa = mic_sens_dbv_pa

    def fs_peak_volts(self, gain_db):
        return self.adc_fs_vrms * math.sqrt(2) / 10 ** (gain_db / 20)

    def amp_fs_to_pa(self, amp_fs, gain_db):
        """Lock-in sine amplitude (fraction of FS peak) -> pressure amplitude in Pa (peak)."""
        v_pk = amp_fs * self.fs_peak_volts(gain_db)
        return v_pk / 10 ** (self.mic_sens_dbv_pa / 20)

    def rms_fs_to_pa(self, rms_fs, gain_db):
        v_rms = rms_fs * self.adc_fs_vrms / 10 ** (gain_db / 20)
        return v_rms / 10 ** (self.mic_sens_dbv_pa / 20)

    @staticmethod
    def pa_to_dbspl(pa_rms):
        return 20 * math.log10(pa_rms / 20e-6) if pa_rms > 0 else -200.0


def dbfs(x):
    return 20 * math.log10(x) if x > 0 else -200.0

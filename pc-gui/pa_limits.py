"""
pa_limits.py - LED/driver safety limits, valid modulation frequencies and
amplitude calibration for the photoacoustic detector GUI.

Sources:
  Ushio SMBB1900D-1100-02 datasheet: IF 1000 mA continuous, IFP 2000 mA at
  10 us / 1 % duty, VF 1.2 V typ (1.6 V at 2 A), PD 1.6 W, Rth(j-s) 10 K/W.
  actoPulserX4-48V: 2 ohm sense (0.5 mA/LSB with the MCP4728 at 4.096 V FS),
  MOSFET current sink from a USB-PD rail (5/9/12/15/20 V).
  TLV320ADC6120: 2 Vrms differential full scale at 0 dB gain (VREF 2.75 V).
  IM73A135 microphone: -38 dBV/Pa sensitivity.
"""

import math

FS = 48000
BLOCK = 4800

LED_MAX_MA = 2000.0
LED_MAX_MA_DC = 1000.0
LED_PULSE_US_FOR_2A = 10.0
LED_DUTY_FOR_2A = 0.01
LED_VF = 1.4                       # V, typical at ~1 A
LED_PD_MAX_W = 1.6
MIN_WIDTH_US = 1.0
MAX_WIDTH_US = 20.0
MOSFET_WARN_W = 2.0                # average dissipation that deserves a warning
PD_VOLTAGES = (5, 9, 12, 15, 20)

# ---- calibration defaults (editable in the GUI) -----------------------------
ADC_FULL_SCALE_VRMS = 2.0          # differential, 0 dB gain
MIC_SENS_DBV_PA = -38.0


def valid_frequencies():
    """Modulation frequencies with an integer period that divides one block."""
    out = []
    for n in range(4, BLOCK + 1):
        if BLOCK % n == 0 and FS % n == 0:
            out.append(FS // n)
    return sorted(set(out))


def allowed_current_ma(freq_hz, width_us):
    duty = freq_hz * width_us * 1e-6
    if width_us <= LED_PULSE_US_FOR_2A and duty <= LED_DUTY_FOR_2A:
        return LED_MAX_MA
    return LED_MAX_MA_DC


def check_settings(freq_hz, width_us, current_ma, pd_volts):
    """
    Returns (ok, warnings, info) for a proposed excitation setting.
    ok=False means the setting must not be sent.
    """
    warnings = []
    info = {}
    duty = freq_hz * width_us * 1e-6
    info["duty"] = duty
    info["avg_led_ma"] = current_ma * duty
    info["avg_led_w"] = current_ma * 1e-3 * LED_VF * duty
    info["mosfet_avg_w"] = max(0.0, pd_volts - LED_VF) * current_ma * 1e-3 * duty
    info["mosfet_peak_w"] = max(0.0, pd_volts - LED_VF) * current_ma * 1e-3
    info["allowed_ma"] = allowed_current_ma(freq_hz, width_us)

    ok = True
    if not MIN_WIDTH_US <= width_us <= MAX_WIDTH_US:
        ok = False
        warnings.append("pulse width must be %.0f..%.0f us (one frame is 20.8 us)"
                        % (MIN_WIDTH_US, MAX_WIDTH_US))
    if current_ma > info["allowed_ma"]:
        ok = False
        warnings.append("LED limit: %.0f mA max at %.1f us / %.2f %% duty"
                        % (info["allowed_ma"], width_us, duty * 100))
    if current_ma > LED_MAX_MA_DC and duty > 0.005:
        warnings.append("above 1 A: keep duty <= 1 %%, currently %.2f %%" % (duty * 100))
    if info["avg_led_w"] > LED_PD_MAX_W:
        ok = False
        warnings.append("LED average power %.2f W exceeds 1.6 W" % info["avg_led_w"])
    if info["mosfet_avg_w"] > MOSFET_WARN_W:
        warnings.append("driver MOSFET dissipates %.1f W average at %d V supply; "
                        "select a lower USB-PD voltage" % (info["mosfet_avg_w"], pd_volts))
    if pd_volts > 9 and current_ma > 0:
        warnings.append("a 1.4 V LED on %d V wastes %.0f%% in the driver; 5 V is enough"
                        % (pd_volts, 100 * (1 - LED_VF / pd_volts)))
    return ok, warnings, info


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

"""
main.py - photoacoustic detector firmware for the RP2040 Pico.

Hardware (see kicad/ and the actoPulserX4-48V project):
    ADC TLV320ADC6120   I2C0  GP4 SDA / GP5 SCL      I2S GP6 BCLK, GP7 FSYNC, GP8 SDOUT
    actopulser MCP4728  I2C1  GP2 SDA / GP3 SCL      LDAC GP14 (held low)
    actopulser PULSEa-d GP10..GP13 (PIO pulser locked to FSYNC)
    Pico onboard LED    GP25

Every 100 ms block (4800 frames at 48 kHz) the Pico computes the lock-in
I/Q at the modulation frequency and reports it as one JSON line. Every
`burst` blocks it also sends the raw channel-1 samples of the last block
(base64 int16) so the PC can draw a spectrum and a time trace.

Line protocol over USB serial (commands are plain text, replies are JSON):
    id                        -> {"type":"id", ...}
    status                    -> {"type":"status", ...}
    selftest                  -> {"type":"selftest","results":[...]}
    set <key> <value>         -> {"type":"ack","key":..,"value":..}
        keys: freq (Hz), width (us), current (mA), channel (0-3), gain (dB),
              burst (blocks between raw bursts, 0 = off), heartbeat (s, 0 = off)
    light on|off              -> ack           (LED current + pulses)
    start | stop              -> ack           (data streaming)
    ping                      -> keeps the heartbeat alive
    dump                      -> {"type":"regs", ...}   (ADC page 0)
Unsolicited:
    {"type":"data", ...}  {"type":"burst", ...}  {"type":"log", ...}  {"type":"error", ...}

Safety: the LED (Ushio SMBB1900D) is rated 1 A continuous and 2 A only for
<= 10 us pulses at <= 1 % duty. Limits are enforced here as well as on the PC,
the light is switched off if the PC stops sending commands (heartbeat), and
on any unhandled exception.
"""

import sys
import select
import time
import json
import binascii
from machine import I2C, I2S, Pin

import TLV320
import MCP4728
import pulser
import lockin

FW_NAME = "pa-detector"
FW_VERSION = "0.2.0"

# ---- pin map --------------------------------------------------------------
PIN_ADC_SDA, PIN_ADC_SCL = 4, 5
PIN_BCLK, PIN_FSYNC, PIN_SDOUT = 6, 7, 8
PIN_DAC_SDA, PIN_DAC_SCL, PIN_LDAC = 2, 3, 14
PULSE_PINS = (10, 11, 12, 13)          # actopulser PULSEa..d
PIN_LED = 25

# ---- audio ----------------------------------------------------------------
FS = 48000
BLOCK = 4800                           # frames per block = 100 ms
BITS = 16

# ---- LED / driver limits ----------------------------------------------------
MAX_MA = 2000.0                        # absolute maximum (10 us, 1 % duty)
MAX_MA_DC = 1000.0                     # otherwise
MAX_PULSE_US_FOR_2A = 10.0
MAX_DUTY_FOR_2A = 0.01
MIN_WIDTH_US = 1.0
MAX_WIDTH_US = 20.0                    # must stay below one frame (20.8 us)
MIN_FREQ = 10
DEFAULT_HEARTBEAT_S = 5


def allowed_current_ma(freq, width_us):
    duty = freq * width_us * 1e-6
    if width_us <= MAX_PULSE_US_FOR_2A and duty <= MAX_DUTY_FOR_2A:
        return MAX_MA
    return MAX_MA_DC


def valid_freq(freq):
    """f_mod must divide fs and one block must hold whole periods."""
    if freq < MIN_FREQ or FS % freq:
        return False
    return BLOCK % (FS // freq) == 0


class App:
    def __init__(self):
        self.led = Pin(PIN_LED, Pin.OUT, value=0)
        self.rx = ""
        self.poll = select.poll()
        self.poll.register(sys.stdin, select.POLLIN)

        # settings
        self.freq = 1000
        self.width_us = 10.0
        self.current_ma = 0.0
        self.channel = 0
        self.gain_db = 12.0
        self.light = False
        self.streaming = True
        self.burst_every = 10
        self.heartbeat_s = DEFAULT_HEARTBEAT_S
        self.last_cmd = time.ticks_ms()
        self.seq = 0
        self.t0 = time.ticks_ms()

        # hardware state
        self.adc_ok = False
        self.adc_defaults = None
        self.dac_ok = False
        self.i2s = None
        self.last = None
        self.status_cache = {}

        self.buf = bytearray(BLOCK * 4)               # int16 stereo frames
        self.left = bytearray(BLOCK * 2)              # int16 channel 1
        self.li = lockin.LockIn(FS // self.freq)

    # ---- output helpers -----------------------------------------------------
    def send(self, obj):
        sys.stdout.write(json.dumps(obj))
        sys.stdout.write("\n")

    def log(self, level, msg):
        self.send({"type": "log", "level": level, "msg": msg})

    def error(self, msg):
        self.send({"type": "error", "msg": msg})

    # ---- hardware ------------------------------------------------------------
    def init_hw(self):
        # ADC
        self.i2c0 = I2C(0, sda=Pin(PIN_ADC_SDA), scl=Pin(PIN_ADC_SCL), freq=400_000)
        self.adc = TLV320.TLV320ADC6120(self.i2c0)
        self.adc_ok = self.adc.detect()
        if self.adc_ok:
            self.adc.reset()
            self.adc_defaults = self.adc.verify()
            self.adc.begin(bits=BITS, gain_db=self.gain_db)
        else:
            self.log("error", "TLV320ADC6120 not found on I2C0")

        # I2S master, always started so FSYNC exists for the pulser
        self.i2s = I2S(0, sck=Pin(PIN_BCLK), ws=Pin(PIN_FSYNC), sd=Pin(PIN_SDOUT),
                       mode=I2S.RX, bits=BITS, format=I2S.STEREO, rate=FS,
                       ibuf=BLOCK * 4 * 4)

        # actopulser DAC (3.3 V I2C into a 5 V MCP4728 works in practice)
        Pin(PIN_DAC_SDA, Pin.IN, Pin.PULL_UP)
        Pin(PIN_DAC_SCL, Pin.IN, Pin.PULL_UP)
        self.i2c1 = I2C(1, sda=Pin(PIN_DAC_SDA), scl=Pin(PIN_DAC_SCL), freq=100_000)
        self.ldac = Pin(PIN_LDAC, Pin.OUT, value=0)
        self.dac = MCP4728.MCP4728(self.i2c1)
        try:
            self.dac_ok = self.dac.present()
            if self.dac_ok:
                self.dac.off_all()
        except OSError:
            self.dac_ok = False
        if not self.dac_ok:
            self.log("warn", "actopulser MCP4728 not found on I2C1")

        # pulser (idle)
        self.pulser = pulser.Pulser(PULSE_PINS[self.channel], FS // self.freq, self.width_us)
        for p in PULSE_PINS:
            if p != PULSE_PINS[self.channel]:
                Pin(p, Pin.IN)

        time.sleep_ms(50)
        self.refresh_status()

    def refresh_status(self):
        if self.adc_ok:
            try:
                self.status_cache = self.adc.status()
            except OSError:
                self.status_cache = {"clock_ok": False, "mode": "i2c error"}
        else:
            self.status_cache = {"clock_ok": False, "mode": "no adc"}

    # ---- light control --------------------------------------------------------
    def apply_current(self):
        if not self.dac_ok:
            return
        code = MCP4728.MCP4728.ma_to_code(self.current_ma) if self.light else 0
        self.dac.write(self.channel, code)

    def set_light(self, on):
        if on and not self.dac_ok:
            raise ValueError("actopulser not present")
        if on:
            self.light = True
            self.apply_current()
            self.pulser.configure(FS // self.freq, self.width_us)
            self.pulser.start()
        else:
            self.pulser.stop()
            self.light = False
            self.apply_current()
        self.led.value(1 if self.light else 0)

    def clamp_current(self):
        lim = allowed_current_ma(self.freq, self.width_us)
        if self.current_ma > lim:
            self.current_ma = lim
            self.log("warn", "current clamped to %.0f mA by LED duty limit" % lim)
            self.apply_current()

    # ---- commands ---------------------------------------------------------------
    def handle(self, line):
        self.last_cmd = time.ticks_ms()
        parts = line.strip().split()
        if not parts:
            return
        cmd = parts[0].lower()
        try:
            if cmd == "ping":
                return
            if cmd == "id":
                self.send({"type": "id", "fw": FW_NAME, "version": FW_VERSION,
                           "fs": FS, "block": BLOCK, "bits": BITS,
                           "adc": self.adc_ok, "dac": self.dac_ok,
                           "ma_per_lsb": MCP4728.MA_PER_LSB,
                           "pulse_pins": PULSE_PINS})
            elif cmd == "status":
                self.refresh_status()
                self.send(self.status_obj())
            elif cmd == "selftest":
                self.send({"type": "selftest", "results": self.selftest()})
            elif cmd == "start":
                self.streaming = True
                self.send({"type": "ack", "key": "stream", "value": True})
            elif cmd == "stop":
                self.streaming = False
                self.send({"type": "ack", "key": "stream", "value": False})
            elif cmd == "light":
                on = len(parts) > 1 and parts[1].lower() in ("on", "1", "true")
                self.set_light(on)
                self.send({"type": "ack", "key": "light", "value": self.light})
            elif cmd == "set" and len(parts) >= 3:
                self.set_param(parts[1].lower(), parts[2])
            elif cmd == "dump":
                regs = {}
                if self.adc_ok:
                    for r in range(0x78):
                        regs["0x%02X" % r] = self.adc.read_reg(r)
                self.send({"type": "regs", "regs": regs})
            else:
                self.error("unknown command: " + line.strip())
        except (ValueError, OSError) as e:
            self.error("%s: %s" % (cmd, e))

    def set_param(self, key, val):
        if key == "freq":
            f = int(float(val))
            if not valid_freq(f):
                raise ValueError("freq must divide 48000 and 4800/period must be integer")
            self.freq = f
            self.li.set_period(FS // f)
            self.pulser.configure(period_frames=FS // f)
            self.clamp_current()
            self.send({"type": "ack", "key": key, "value": self.freq})
        elif key == "width":
            w = float(val)
            if not MIN_WIDTH_US <= w <= MAX_WIDTH_US:
                raise ValueError("width %.1f..%.1f us" % (MIN_WIDTH_US, MAX_WIDTH_US))
            self.width_us = w
            self.pulser.configure(width_us=w)
            self.clamp_current()
            self.send({"type": "ack", "key": key, "value": self.pulser.actual_width_us()})
        elif key == "current":
            ma = float(val)
            if ma < 0:
                raise ValueError("current >= 0")
            lim = allowed_current_ma(self.freq, self.width_us)
            if ma > lim:
                raise ValueError("current above LED limit of %.0f mA for this width/duty" % lim)
            self.current_ma = ma
            self.apply_current()
            self.send({"type": "ack", "key": key,
                       "value": MCP4728.MCP4728.code_to_ma(MCP4728.MCP4728.ma_to_code(ma))})
        elif key == "channel":
            ch = int(val)
            if not 0 <= ch <= 3:
                raise ValueError("channel 0..3")
            was = self.light
            if was:
                self.set_light(False)
            self.channel = ch
            self.pulser.set_pin(PULSE_PINS[ch])
            if was:
                self.set_light(True)
            self.send({"type": "ack", "key": key, "value": ch})
        elif key == "gain":
            g = float(val)
            if not self.adc_ok:
                raise ValueError("no ADC")
            self.adc.set_gain(1, g)
            self.gain_db = round(g * 2) / 2
            self.send({"type": "ack", "key": key, "value": self.gain_db})
        elif key == "burst":
            self.burst_every = max(0, int(val))
            self.send({"type": "ack", "key": key, "value": self.burst_every})
        elif key == "heartbeat":
            self.heartbeat_s = max(0, int(val))
            self.send({"type": "ack", "key": key, "value": self.heartbeat_s})
        else:
            raise ValueError("unknown key " + key)

    def status_obj(self):
        return {"type": "status", "adc": self.status_cache, "adc_ok": self.adc_ok,
                "dac_ok": self.dac_ok, "freq": self.freq, "width_us": self.pulser.actual_width_us(),
                "current_ma": self.current_ma, "channel": self.channel, "gain_db": self.gain_db,
                "light": self.light, "streaming": self.streaming, "burst": self.burst_every,
                "heartbeat_s": self.heartbeat_s, "seq": self.seq}

    # ---- self test --------------------------------------------------------------
    def selftest(self):
        res = []

        def add(name, status, detail=""):
            res.append({"name": name, "status": status, "detail": detail})

        # ADC presence
        try:
            devs = self.i2c0.scan()
            add("adc_i2c_ack", "pass" if TLV320.I2C_ADDR in devs else "fail",
                "I2C0 devices: " + ", ".join("0x%02X" % d for d in devs))
        except OSError as e:
            add("adc_i2c_ack", "fail", str(e))
        add("adc_register_test", "pass" if self.adc_ok and self.adc.detect() else "fail",
            "page register write/read-back")
        if self.adc_defaults is None:
            add("adc_reset_defaults", "skip", "ADC not initialised")
        else:
            add("adc_reset_defaults", "pass" if not self.adc_defaults else "fail",
                "mismatches: %s" % self.adc_defaults)

        # clocks and mode
        self.refresh_status()
        st = self.status_cache
        add("adc_clock_detect", "pass" if st.get("clock_ok") else "fail",
            "fs=%s ratio=%s" % (st.get("fs"), st.get("bclk_ratio")))
        add("adc_active_mode", "pass" if "channel(s) on" in str(st.get("mode")) else "fail",
            "mode=%s ch1=%s" % (st.get("mode"), st.get("ch1_on")))

        # FSYNC toggling (needed by the pulser)
        p = Pin(PIN_FSYNC)
        seen = 0
        for _ in range(400):
            seen |= 1 << p.value()
        add("fsync_toggling", "pass" if seen == 3 else "fail",
            "I2S word clock observed on GP%d" % PIN_FSYNC)

        # microphone noise floor
        if self.last is None:
            add("mic_noise_floor", "skip", "no audio block yet")
        else:
            rms_db = lockin.LockIn.dbfs(self.last[4])
            if rms_db < -95:
                s, d = "fail", "input silent (%.1f dBFS): mic dead, unpowered or open" % rms_db
            elif rms_db > -20:
                s, d = "warn", "very loud (%.1f dBFS): overload, oscillation or hum" % rms_db
            else:
                s, d = "pass", "%.1f dBFS at %.1f dB gain" % (rms_db, self.gain_db)
            add("mic_noise_floor", s, d)
            add("adc_overload", "pass" if self.last[5] < 32000 else "warn",
                "peak %d / 32767" % self.last[5])

        # actopulser DAC
        try:
            devs = self.i2c1.scan()
            add("dac_i2c_ack", "pass" if MCP4728.I2C_ADDR in devs else "fail",
                "I2C1 devices: " + ", ".join("0x%02X" % d for d in devs))
        except OSError as e:
            add("dac_i2c_ack", "fail", str(e))
        if self.dac_ok:
            try:
                rb = self.dac.read()
                exp = MCP4728.MCP4728.ma_to_code(self.current_ma) if self.light else 0
                got = rb[self.channel]["code"]
                add("dac_readback", "pass" if got == exp else "fail",
                    "channel %d code %d (expected %d), vref=%d gain=%d" %
                    (self.channel, got, exp, rb[self.channel]["vref"], rb[self.channel]["gain"]))
            except OSError as e:
                add("dac_readback", "fail", str(e))
        else:
            add("dac_readback", "skip", "actopulser not present")

        add("pulser_state", "pass", "PIO %s on GP%d, period %d frames, width %.1f us" %
            ("running" if self.pulser.active else "idle", self.pulser.pin_no,
             self.pulser.period, self.pulser.actual_width_us()))
        add("light_functional", "skip", "run the light on/off check from the PC")
        return res

    # ---- main loop ---------------------------------------------------------------
    def poll_cmds(self):
        while self.poll.poll(0):
            c = sys.stdin.read(1)
            if not c:
                break
            if c in "\r\n":
                if self.rx:
                    line, self.rx = self.rx, ""
                    self.handle(line)
            else:
                self.rx += c
                if len(self.rx) > 200:
                    self.rx = ""

    def check_heartbeat(self):
        if self.light and self.heartbeat_s and \
                time.ticks_diff(time.ticks_ms(), self.last_cmd) > self.heartbeat_s * 1000:
            self.set_light(False)
            self.log("warn", "heartbeat lost: light switched off")

    def send_burst(self):
        lockin.deinterleave_left(self.buf, self.left, BLOCK)
        b64 = binascii.b2a_base64(self.left)[:-1].decode()
        self.send({"type": "burst", "seq": self.seq, "n": BLOCK, "fs": FS,
                   "period": self.li.period, "b64": b64})

    def run(self):
        self.init_hw()
        self.send({"type": "id", "fw": FW_NAME, "version": FW_VERSION, "fs": FS,
                   "block": BLOCK, "bits": BITS, "adc": self.adc_ok, "dac": self.dac_ok,
                   "ma_per_lsb": MCP4728.MA_PER_LSB, "pulse_pins": PULSE_PINS})
        while True:
            n = self.i2s.readinto(self.buf)
            if n < len(self.buf):
                self.poll_cmds()
                continue
            self.seq += 1
            i, q, amp, ph, rms, peak = self.li.process(self.buf, BLOCK)
            self.last = (i, q, amp, ph, rms, peak)
            if self.seq % 10 == 0:
                self.refresh_status()
            if self.streaming:
                self.send({"type": "data", "seq": self.seq,
                           "t": time.ticks_diff(time.ticks_ms(), self.t0),
                           "i": i, "q": q, "amp": amp, "amp_db": lockin.LockIn.dbfs(amp),
                           "ph": ph, "rms": rms, "rms_db": lockin.LockIn.dbfs(rms),
                           "peak": peak, "ovl": peak >= 32000,
                           "clk": bool(self.status_cache.get("clock_ok")),
                           "freq": self.freq, "light": self.light,
                           "ma": self.current_ma if self.light else 0.0,
                           "gain": self.gain_db})
                if self.burst_every and self.seq % self.burst_every == 0:
                    self.send_burst()
            if not self.light:
                self.led.value(self.seq % 10 < 5)
            self.poll_cmds()
            self.check_heartbeat()


app = App()
try:
    app.run()
except KeyboardInterrupt:
    pass
except Exception as e:
    try:
        app.error("fatal: %r" % e)
    except Exception:
        pass
    raise
finally:
    try:
        app.set_light(False)
    except Exception:
        pass
    try:
        if app.dac_ok:
            app.dac.off_all()
    except Exception:
        pass

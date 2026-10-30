"""
main.py - photoacoustic detector firmware for the RP2040 Pico.

Hardware (see kicad/):
    ADC TLV320ADC6120   I2C0  GP4 SDA / GP5 SCL      I2S GP6 BCLK, GP7 FSYNC, GP8 SDOUT
    LED current source  GP0 (Pico pin 1) control input, on/off: high = LED on
    Pico onboard LED    GP25

The LED is square-wave modulated (50 % duty) by a PIO state machine that counts
I2S bit clocks (modulator.py), so f_mod = 64 * fs / P for an integer P and the
lock-in reference stays phase locked to the light. Every 100 ms block (4800
frames at 48 kHz) the Pico computes the lock-in I/Q at f_mod and reports it as
one JSON line. Every `burst` blocks it also sends the raw channel-1 samples of
the block (base64 int16); with burst 1 (default) the PC gets the gap-free
sample stream for its FFT.

Line protocol over USB serial (commands are plain text, replies are JSON):
    id                        -> {"type":"id", ...}
    status                    -> {"type":"status", ...}
    selftest                  -> {"type":"selftest","results":[...]}
    set <key> <value>         -> {"type":"ack","key":..,"value":..}
        keys: freq (Hz, 200..10000, snapped to 64*fs/P; the ack carries the
              actual frequency and p_bclk), gain (dB),
              burst (blocks between raw bursts, 0 = off), heartbeat (s, 0 = off)
    light on|off              -> ack           (modulation on GP0)
    start | stop              -> ack           (data streaming)
    ping                      -> keeps the heartbeat alive
    dump                      -> {"type":"regs", ...}   (ADC page 0)
Unsolicited:
    {"type":"data", ...}  {"type":"burst", ...}  {"type":"log", ...}  {"type":"error", ...}

Safety: the LED (Ushio SMBB1900D, 1 A continuous) gets at most 1 A, set in the
current source hardware, at 50 % duty. GP0 is driven low first thing at boot,
the light is switched off if the PC stops sending commands (heartbeat), and on
any unhandled exception.
"""

from machine import I2C, I2S, Pin

Pin(0, Pin.OUT, value=0)               # PIN_MOD: LED current source off before anything else

import gc
import sys
import select
import time
import json
import binascii

import TLV320
import modulator
import lockin

FW_NAME = "pa-detector"
FW_VERSION = "0.3.0"

# ---- pin map --------------------------------------------------------------
PIN_MOD = 0                            # current source control, Pico pin 1
PIN_ADC_SDA, PIN_ADC_SCL = 4, 5
PIN_BCLK, PIN_FSYNC, PIN_SDOUT = 6, 7, 8
PIN_LED = 25

# ---- audio ----------------------------------------------------------------
FS = 48000
BLOCK = 4800                           # frames per block = 100 ms
BITS = 16
B64_CHUNK = 2400                       # bytes per base64 piece of a burst (multiple of 3)

# ---- modulation -------------------------------------------------------------
BCLK_PER_FRAME = modulator.BCLK_PER_FRAME
F_MIN, F_MAX = 200.0, 10000.0
P_MIN = -(-BCLK_PER_FRAME * FS // int(F_MAX))  # 308 bit clocks = 9974 Hz (ceil: stay <= F_MAX)
P_MAX = BCLK_PER_FRAME * FS // int(F_MIN)      # 15360 bit clocks = 200 Hz
DEFAULT_HEARTBEAT_S = 5


class App:
    def __init__(self):
        self.led = Pin(PIN_LED, Pin.OUT, value=0)
        self.rx = ""
        self.poll = select.poll()
        self.poll.register(sys.stdin, select.POLLIN)

        # settings
        self.p_bclk = BCLK_PER_FRAME * FS // 1000   # 1 kHz
        self.freq = BCLK_PER_FRAME * FS / self.p_bclk
        self.gain_db = 12.0
        self.light = False
        self.streaming = True
        self.burst_every = 1
        self.heartbeat_s = DEFAULT_HEARTBEAT_S
        self.last_cmd = time.ticks_ms()
        self.seq = 0
        self.t0 = time.ticks_ms()

        # hardware state
        self.adc_ok = False
        self.adc_defaults = None
        self.i2s = None
        self.mod = None
        self.bclk_per_frame = 0                       # measured on the bus, see init_hw
        self.last = None
        self.status_cache = {}
        self.buf = None                               # allocated after the I2S buffer, see init_hw
        self.left = None
        self.li = None

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

        # I2S master, always started so BCLK exists for the modulator. Its 76.8 kB ring
        # buffer must be one contiguous block, so it is allocated before anything else
        # big: allocated earlier, the sample buffers split the heap and this fails.
        gc.collect()
        self.i2s = I2S(0, sck=Pin(PIN_BCLK), ws=Pin(PIN_FSYNC), sd=Pin(PIN_SDOUT),
                       mode=I2S.RX, bits=BITS, format=I2S.STEREO, rate=FS,
                       ibuf=BLOCK * 4 * 4)
        self.buf = bytearray(BLOCK * 4)               # int16 stereo frames
        self.left = bytearray(BLOCK * 2)              # int16 channel 1
        self.left_mv = memoryview(self.left)
        self.li = lockin.LockIn(self.p_bclk, BCLK_PER_FRAME)

        # LED modulator (idle, GP0 low). Its frequency scale assumes BCLK_PER_FRAME.
        self.mod = modulator.Modulator(PIN_MOD, self.p_bclk)
        self.bclk_per_frame = modulator.measure_bclk_per_frame()
        if self.bclk_per_frame != BCLK_PER_FRAME:
            self.log("error", "I2S frame is %d bit clocks, expected %d: modulation frequency is wrong"
                     % (self.bclk_per_frame, BCLK_PER_FRAME))

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
    def set_light(self, on):
        if on:
            self.mod.configure(self.p_bclk)
            self.mod.start()
        else:
            self.mod.stop()
        self.light = bool(on)
        self.led.value(1 if self.light else 0)

    def set_period(self, p):
        p = max(P_MIN, min(P_MAX, int(p)))
        self.p_bclk = p
        self.freq = BCLK_PER_FRAME * FS / p
        self.mod.configure(p)
        self.li.set_period(p)

    # ---- commands ---------------------------------------------------------------
    def ident(self):
        return {"type": "id", "fw": FW_NAME, "version": FW_VERSION, "fs": FS,
                "block": BLOCK, "bits": BITS, "adc": self.adc_ok, "driver": "vccs",
                "mod_pin": PIN_MOD, "bclk_per_frame": BCLK_PER_FRAME,
                "fmin": F_MIN, "fmax": F_MAX}

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
                self.send(self.ident())
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
            f = float(val)
            if not F_MIN <= f <= F_MAX:
                raise ValueError("freq %.0f..%.0f Hz" % (F_MIN, F_MAX))
            self.set_period(round(BCLK_PER_FRAME * FS / f))
            self.send({"type": "ack", "key": key, "value": self.freq, "p_bclk": self.p_bclk})
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
                "freq": self.freq, "p_bclk": self.p_bclk, "gain_db": self.gain_db,
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

        # I2S clocks toggling (BCLK clocks the modulator, FSYNC frames the samples)
        for name, pin_no in (("bclk_toggling", PIN_BCLK), ("fsync_toggling", PIN_FSYNC)):
            p = Pin(pin_no)
            seen = 0
            for _ in range(400):
                seen |= 1 << p.value()
            add(name, "pass" if seen == 3 else "fail", "I2S clock observed on GP%d" % pin_no)
        self.bclk_per_frame = modulator.measure_bclk_per_frame()
        add("bclk_per_frame", "pass" if self.bclk_per_frame == BCLK_PER_FRAME else "fail",
            "%d bit clocks per frame (modulator assumes %d)" % (self.bclk_per_frame, BCLK_PER_FRAME))

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

        # modulator output: idle low, or toggling (one period is at most 5 ms)
        p = Pin(PIN_MOD)
        seen = 0
        t0 = time.ticks_ms()
        while time.ticks_diff(time.ticks_ms(), t0) < 12:
            seen |= 1 << p.value()
        if self.light:
            add("mod_pin", "pass" if seen == 3 else "fail",
                "GP%d %s at %.1f Hz" % (PIN_MOD, "toggling" if seen == 3 else "stuck", self.freq))
        else:
            add("mod_pin", "pass" if seen == 1 else "fail",
                "GP%d %s with the light off" % (PIN_MOD, "low" if seen == 1 else "NOT low"))
        add("modulator_state", "pass", "PIO %s, P = %d bit clocks (%.3f Hz), 50 %% duty" %
            ("running" if self.mod.active else "idle", self.p_bclk, self.freq))
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
        # base64 in 3.2 kB pieces written straight out: ten 13 kB strings per second
        # would fragment the heap (pieces of a multiple of 3 bytes concatenate exactly)
        lockin.deinterleave_left(self.buf, self.left, BLOCK)
        w = sys.stdout.write
        w('{"type":"burst","seq":%d,"n":%d,"fs":%d,"p_bclk":%d,"ph0":%.6f,"b64":"' %
          (self.seq, BLOCK, FS, self.li.period, self.li.ph0))
        mv = self.left_mv
        for k in range(0, BLOCK * 2, B64_CHUNK):
            w(binascii.b2a_base64(mv[k:k + B64_CHUNK], newline=False))
        w('"}\n')

    def run(self):
        self.init_hw()
        self.send(self.ident())
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
                           "freq": self.freq, "light": self.light, "gain": self.gain_db})
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
    Pin(PIN_MOD, Pin.OUT, value=0)     # back to plain GPIO low, whatever state the PIO is in

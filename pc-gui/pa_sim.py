"""
pa_sim.py - a simulated photoacoustic detector, to try the GUI and to test it without hardware.

    py pa_gui.py --sim          the port list gets a "SIM" entry: connect to it like to the real board

SimSerial stands in for serial.Serial. It answers the firmware's commands (firmware/src/protocol.c) with the
same JSON lines, and while streaming sends, every 100 ms, a data line and then the block's raw samples, in
that order, like the firmware. The lock-in follows firmware/src/lockin.c: reference phase ph0 + n * 64 / P
cycles, DC removed, the phase reported as the lag of the signal behind the LED rising edge. The light
switches off when no command has arrived for heartbeat_s seconds.

The cell (CellModel) is a resonance at f0 with quality factor q on a 1/f background, delayed by the ADC's
group delay, plus white noise; the LED's square wave also excites the 3rd and 5th harmonics. With the
modulation off only the noise is left. PA_SIM_SPEED (environment) runs the clock faster, e.g. 10 for a block
every 10 ms.
"""

import base64
import json
import math
import os
import threading
import time

import numpy as np

import pa_limits

PORT = "SIM"
FW_VERSION = "1.0.1-sim"
ADC_LATENCY_SAMPLES = 17.1
HPF_HZ = 12
enabled = False


def enable(on=True):
    """List the simulated port (pa_device.list_ports) and let PADevice open it."""
    global enabled
    enabled = on


def is_sim_port(port):
    return enabled and port == PORT


class CellModel:
    def __init__(self, f0=3000.0, q=12.0, peak_fs=3e-3, background_fs=2e-4, noise_rms_fs=6e-5, seed=1):
        self.f0, self.q = f0, q
        self.peak_fs, self.background_fs, self.noise_rms_fs = peak_fs, background_fs, noise_rms_fs
        self.rng = np.random.default_rng(seed)

    def response(self, f, gain_db):
        """Complex amplitude of the microphone signal at f, fraction of full scale; angle = -lag."""
        g = 10 ** ((gain_db - 12.0) / 20.0)
        res = self.peak_fs / (1 + 1j * self.q * (f / self.f0 - self.f0 / f))
        bg = self.background_fs * (1000.0 / f) * np.exp(-0.5j * math.pi)
        delay = np.exp(-2j * math.pi * f * ADC_LATENCY_SAMPLES / pa_limits.FS)
        return complex(g * (res + bg) * delay)

    def expected(self, f, gain_db):
        """(amp_fs, phase_deg as the firmware reports it) of the fundamental."""
        h = self.response(f, gain_db)
        return abs(h), -math.degrees(math.atan2(h.imag, h.real))


class SimSerial:
    def __init__(self, port=PORT, model=None, speed=None):
        self.port = port
        self.model = model or CellModel()
        self.speed = float(speed or os.environ.get("PA_SIM_SPEED", "1") or 1)
        self.timeout = 0.2
        self._out = bytearray()
        self._cond = threading.Condition()
        self._inbuf = b""
        self.is_open = True
        # firmware state
        self.streaming = True
        self.light = False
        self.p, self.freq = pa_limits.snap_frequency(1000.0)
        self.gain = 0.0
        self.burst_every = 1
        self.heartbeat_s = 5
        self.seq = 0
        self.mod_seq = 0
        self.ph0 = 0.0
        self.last_cmd = time.time()
        self.t_start = time.time()
        self._stop = threading.Event()
        self._thread = threading.Thread(target=self._run, daemon=True)
        self._thread.start()

    # ---- serial.Serial interface used by PADevice
    def read(self, n=1):
        with self._cond:
            if not self._out and self.is_open:
                self._cond.wait(self.timeout)
            if not self.is_open:
                raise OSError("simulated port closed")
            chunk = bytes(self._out[:n])
            del self._out[:n]
            return chunk

    def write(self, data):
        if not self.is_open:
            raise OSError("simulated port closed")
        self._inbuf += bytes(data)
        while b"\n" in self._inbuf:
            line, self._inbuf = self._inbuf.split(b"\n", 1)
            self._command(line.decode(errors="replace").strip())
        return len(data)

    def close(self):
        self._stop.set()
        with self._cond:
            self.is_open = False
            self._cond.notify_all()

    # ---- firmware behaviour
    def _send(self, obj):
        with self._cond:
            self._out += (json.dumps(obj, separators=(",", ":")) + "\n").encode()
            self._cond.notify_all()

    def _ack(self, key, value, **extra):
        self._send(dict({"type": "ack", "key": key, "value": value}, **extra))

    def _set_light(self, on):
        if on != self.light:
            self.light = on
            self.mod_seq += 1

    def _command(self, line):
        self.last_cmd = time.time()
        tok = line.split()
        if not tok:
            return
        cmd = tok[0].lower()
        if cmd == "ping":
            return
        if cmd == "id":
            self._send({"type": "id", "fw": "pa-detector", "version": FW_VERSION, "proto": 2,
                        "fs": pa_limits.FS, "block": pa_limits.BLOCK, "bits": 16, "adc_bits": 24, "adc": True,
                        "driver": "vccs", "mod_pin": 0, "bclk_per_frame": pa_limits.BCLK_PER_FRAME,
                        "fmin": pa_limits.F_MIN, "fmax": pa_limits.F_MAX, "sysclk_hz": 76800000,
                        "hpf_hz": HPF_HZ, "adc_latency_samples": ADC_LATENCY_SAMPLES, "mod_edge_frames": 0.0,
                        "table_len": 4096, "uid": "SIMULATED", "reset_reason": "power/reset"})
        elif cmd == "status":
            self._send({"type": "status",
                        "adc": {"mode": "active", "ch1_on": True, "ch2_on": False, "fs": "48 kHz",
                                "bclk_ratio": 64, "clock_ok": True, "int_latch": 0},
                        "adc_ok": True, "freq": round(self.freq, 4), "p_bclk": self.p, "gain_db": self.gain,
                        "light": self.light, "streaming": self.streaming, "burst": self.burst_every,
                        "heartbeat_s": self.heartbeat_s, "seq": self.seq, "overruns": 0, "blocks_skipped": 0,
                        "i2c_errors": 0, "tx_drop_burst": 0, "tx_drop_data": 0, "mod_sync": "locked",
                        "mod_seq": self.mod_seq, "block_period_us": 100000,
                        "uptime_s": int(time.time() - self.t_start), "hpf_hz": HPF_HZ, "ch2": False,
                        "sysclk_hz": 76800000, "mod_w_entry": 0, "mod_w_armed": 0, "mod_arm_us": 0})
        elif cmd == "selftest":
            self._send({"type": "selftest", "results": [
                {"name": n, "status": "pass", "detail": "simulated"}
                for n in ("adc_i2c", "adc_clock", "lockin_math", "modulator", "mic_noise_floor")]})
        elif cmd in ("start", "stop"):
            self.streaming = cmd == "start"
            self._ack("stream", self.streaming)
        elif cmd == "light":
            self._set_light(len(tok) > 1 and tok[1].lower() in ("on", "1", "true"))
            self._ack("light", self.light)
        elif cmd == "set" and len(tok) >= 3:
            key, val = tok[1].lower(), tok[2]
            try:
                v = float(val)
            except ValueError:
                self._send({"type": "error", "msg": "bad value %s" % val})
                return
            if key == "freq":
                if not pa_limits.F_MIN <= v <= pa_limits.F_MAX:
                    self._send({"type": "error", "msg": "freq %.0f..%.0f Hz" % (pa_limits.F_MIN, pa_limits.F_MAX)})
                    return
                self.p, self.freq = pa_limits.snap_frequency(v)
                self.mod_seq += 1
                self._ack("freq", round(self.freq, 4), p_bclk=self.p)
            elif key == "gain":
                code = round(v * 2)
                if not 0 <= code <= 84:
                    self._send({"type": "error", "msg": "gain 0..42 dB"})
                    return
                self.gain = code / 2.0
                self._ack("gain", self.gain)
            elif key == "burst":
                self.burst_every = max(0, int(v))
                self._ack("burst", self.burst_every)
            elif key == "heartbeat":
                self.heartbeat_s = max(0, int(v))
                self._ack("heartbeat", self.heartbeat_s)
            else:
                self._send({"type": "error", "msg": "unknown key %s" % key})
        elif cmd == "dump":
            self._send({"type": "regs", "regs": {}})
        elif cmd in ("bootsel", "reset"):
            self._set_light(False)
            self._ack(cmd, True)
        else:
            self._send({"type": "error", "msg": "unknown command: %s" % line})

    def make_block(self):
        """(data message, burst message) of the next 100 ms block."""
        n_s, fs, p = pa_limits.BLOCK, pa_limits.FS, self.p
        n = np.arange(n_s)
        th = 2 * np.pi * (self.ph0 + n * pa_limits.BCLK_PER_FRAME / p)
        g = 10 ** ((self.gain - 12.0) / 20.0)
        x = self.model.rng.normal(0.0, self.model.noise_rms_fs * g, n_s)
        if self.light:
            for h in (1, 3, 5):
                if h * self.freq >= fs / 2:
                    break
                r = self.model.response(h * self.freq, self.gain)
                x += abs(r) / h * np.cos(h * th + math.atan2(r.imag, r.real))
        x = np.clip(x, -1.0, 1.0 - 2 ** -23)
        c, s = np.cos(th), np.sin(th)
        i = float(np.dot(x, c) - x.mean() * c.sum())
        q = float(np.dot(x, s) - x.mean() * s.sum())
        amp = 2 * math.hypot(i, q) / n_s
        rms = float(np.sqrt(np.mean(x * x)))
        x16 = np.clip(np.floor(x * 32768.0), -32768, 32767).astype("<i2")
        seq = self.seq
        t_ms = seq * 100
        data = {"type": "data", "seq": seq, "t": t_ms, "i": round(i * 16383 * 32768, 1),
                "q": round(q * 16383 * 32768, 1), "amp": amp, "amp_db": round(pa_limits.dbfs(amp), 2),
                "ph": round(math.degrees(math.atan2(q, i)), 3), "rms": rms, "rms_db": round(pa_limits.dbfs(rms), 2),
                "peak": int(np.max(np.abs(x16.astype(np.int32)))), "ovl": bool(np.max(np.abs(x)) >= 0.98),
                "clk": True, "freq": round(self.freq, 4), "light": self.light, "gain": self.gain,
                "dc": float(x.mean()), "mod_seq": self.mod_seq, "t_us": t_ms * 1000}
        burst = {"type": "burst", "seq": seq, "n": n_s, "fs": fs, "p_bclk": p, "ph0": round(self.ph0, 6),
                 "b64": base64.b64encode(x16.tobytes()).decode()}
        self.ph0 = (self.ph0 + n_s * pa_limits.BCLK_PER_FRAME / p) % 1.0
        self.seq += 1
        return data, burst

    def _run(self):
        period = 0.1 / self.speed
        t_next = time.time() + period
        while not self._stop.is_set():
            delay = t_next - time.time()
            if delay > 0:
                self._stop.wait(delay)
                continue
            t_next += period
            if self.light and self.heartbeat_s and time.time() - self.last_cmd > self.heartbeat_s:
                self._set_light(False)
                self._send({"type": "log", "level": "warn", "msg": "heartbeat lost: light switched off"})
            if not self.streaming:
                continue
            data, burst = self.make_block()
            self._send(data)
            if self.burst_every and data["seq"] % self.burst_every == 0:
                self._send(burst)

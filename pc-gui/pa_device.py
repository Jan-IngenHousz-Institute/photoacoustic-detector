"""
pa_device.py - serial link to the photoacoustic detector firmware (firmware/main.py).

Discovers Picos (USB VID 0x2E8A), detects a Pico left in BOOTSEL mode (RPI-RP2
drive), and runs a reader thread that turns the firmware's JSON lines into Qt
signals. Commands are plain text lines, see firmware/main.py.
"""

import base64
import json
import os
import threading
import time

import numpy as np
import serial
import serial.tools.list_ports
from PySide6.QtCore import QObject, Signal

PICO_VID = 0x2E8A
MICROPYTHON_PID = 0x0005


def list_ports():
    """[(device, description, is_pico, is_micropython)] sorted with Picos first."""
    out = []
    for p in serial.tools.list_ports.comports():
        is_pico = p.vid == PICO_VID
        is_mpy = is_pico and p.pid == MICROPYTHON_PID
        out.append((p.device, p.description or "", is_pico, is_mpy))
    out.sort(key=lambda t: (not t[3], not t[2], t[0]))
    return out


def find_bootsel_drive():
    """Drive letter of a Pico in BOOTSEL mode (RPI-RP2 mass storage), or None."""
    if os.name != "nt":
        for root in ("/media", "/run/media", "/Volumes"):
            if os.path.isdir(root):
                for dp, dn, fn in os.walk(root):
                    if "INFO_UF2.TXT" in fn:
                        return dp
                    dn[:] = dn[:20]
        return None
    for letter in "DEFGHIJKLMNOPQRSTUVWXYZ":
        if os.path.exists("%s:/INFO_UF2.TXT" % letter):
            return letter + ":"
    return None


class PADevice(QObject):
    ident = Signal(dict)
    data = Signal(dict)
    burst = Signal(object, dict)       # np.int16 array, meta
    status = Signal(dict)
    selftest = Signal(list)
    ack = Signal(dict)
    error = Signal(str)
    log = Signal(dict)
    regs = Signal(dict)
    raw = Signal(str)                  # anything that was not JSON (REPL output etc.)
    connected = Signal(bool)

    def __init__(self):
        super().__init__()
        self.ser = None
        self._thread = None
        self._stop = threading.Event()
        self._lock = threading.Lock()
        self.port = None

    # ---- connection ---------------------------------------------------------
    def open(self, port, baud=115200):
        self.close()
        self.ser = serial.Serial(port, baud, timeout=0.2, write_timeout=1.0)
        self.port = port
        self._stop.clear()
        self._thread = threading.Thread(target=self._reader, daemon=True)
        self._thread.start()
        self.connected.emit(True)
        # if the board sits at the REPL (no main.py) this does no harm
        self.send("id")

    def close(self):
        if self.ser is None:
            return
        self._stop.set()
        try:
            self.send("light off")
        except Exception:
            pass
        if self._thread is not None:
            self._thread.join(timeout=1.0)
        try:
            self.ser.close()
        except Exception:
            pass
        self.ser = None
        self.port = None
        self.connected.emit(False)

    def is_open(self):
        return self.ser is not None and self.ser.is_open

    def send(self, cmd):
        if not self.is_open():
            return False
        with self._lock:
            try:
                self.ser.write((cmd.strip() + "\n").encode())
                return True
            except (serial.SerialException, OSError) as e:
                self.error.emit("write failed: %s" % e)
                return False

    # ---- reader thread ----------------------------------------------------------
    def _reader(self):
        buf = b""
        while not self._stop.is_set():
            try:
                chunk = self.ser.read(4096)
            except (serial.SerialException, OSError, TypeError) as e:
                if not self._stop.is_set():
                    self.error.emit("serial read failed: %s" % e)
                    self.connected.emit(False)
                return
            if not chunk:
                continue
            buf += chunk
            while b"\n" in buf:
                line, buf = buf.split(b"\n", 1)
                self._dispatch(line.strip())
            if len(buf) > 200_000:      # runaway garbage
                buf = b""

    def _dispatch(self, line):
        if not line:
            return
        if line[:1] != b"{":
            self.raw.emit(line.decode(errors="replace"))
            return
        try:
            msg = json.loads(line.decode(errors="replace"))
        except ValueError:
            self.raw.emit(line.decode(errors="replace"))
            return
        t = msg.get("type")
        if t == "data":
            self.data.emit(msg)
        elif t == "burst":
            try:
                samples = np.frombuffer(base64.b64decode(msg.pop("b64")), dtype="<i2")
            except (ValueError, KeyError):
                return
            self.burst.emit(samples, msg)
        elif t == "id":
            self.ident.emit(msg)
        elif t == "status":
            self.status.emit(msg)
        elif t == "selftest":
            self.selftest.emit(msg.get("results", []))
        elif t == "ack":
            self.ack.emit(msg)
        elif t == "error":
            self.error.emit(msg.get("msg", "?"))
        elif t == "log":
            self.log.emit(msg)
        elif t == "regs":
            self.regs.emit(msg.get("regs", {}))
        else:
            self.raw.emit(line.decode(errors="replace"))

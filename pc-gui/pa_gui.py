"""
pa_gui.py - desktop GUI for the photoacoustic detector (Pico + TLV320ADC6120 shield
+ actopulser LED driver).

    python pa_gui.py

Features: Pico discovery (incl. BOOTSEL detection), firmware handshake, layered
self-test, excitation control with LED/driver limits, lock-in amplitude/phase
strip charts with averaging and dark subtraction, spectrum with harmonic markers
and SNR, time-domain and pulse-synchronous views, frequency sweep, CSV logging
with full settings context, config save/load, heartbeat safety.
"""

import csv
import json
import math
import os
import sys
import time
from collections import deque

import numpy as np
import pyqtgraph as pg
from PySide6.QtCore import Qt, QTimer, QSettings
from PySide6.QtGui import QFont, QColor
from PySide6.QtWidgets import (
    QApplication, QMainWindow, QWidget, QLabel, QPushButton, QComboBox,
    QDoubleSpinBox, QSpinBox, QCheckBox, QGroupBox, QFormLayout, QVBoxLayout,
    QHBoxLayout, QTabWidget, QPlainTextEdit, QFileDialog, QMessageBox, QDialog,
    QTableWidget, QTableWidgetItem, QLineEdit, QSplitter, QScrollArea, QHeaderView,
    QSizePolicy,
)

import pa_limits
from pa_device import PADevice, list_ports, find_bootsel_drive

pg.setConfigOptions(antialias=False, background="w", foreground="k")

CHANNEL_NAMES = ("a (GP10)", "b (GP11)", "c (GP12)", "d (GP13)")
HISTORY_S = 120
STATUS_COLORS = {"pass": "#2e7d32", "fail": "#c62828", "warn": "#ef6c00",
                 "skip": "#757575", "?": "#9e9e9e"}


def led_label(text):
    lab = QLabel(text)
    lab.setAlignment(Qt.AlignCenter)
    lab.setMinimumWidth(70)
    lab.setStyleSheet("QLabel{border-radius:4px;padding:3px;color:white;background:#9e9e9e;font-weight:bold}")
    return lab


def set_led(lab, state):
    lab.setStyleSheet("QLabel{border-radius:4px;padding:3px;color:white;font-weight:bold;background:%s}"
                      % STATUS_COLORS.get(state, "#9e9e9e"))


class Sequence:
    """Runs a list of (callable, blocks_to_wait) steps driven by incoming data blocks."""

    def __init__(self, steps, on_done, collect=None):
        self.steps = list(steps)
        self.on_done = on_done
        self.collect = collect          # callable(msg) storing values while waiting
        self.remaining = 0
        self.active = False

    def start(self):
        self.active = True
        self._advance()

    def _advance(self):
        if not self.steps:
            self.active = False
            self.on_done()
            return
        fn, n = self.steps.pop(0)
        fn()
        self.remaining = n

    def feed(self, msg):
        if not self.active:
            return
        if self.remaining > 0:
            if self.collect:
                self.collect(msg)
            self.remaining -= 1
        if self.remaining <= 0:
            self._advance()

    def abort(self):
        self.active = False
        self.steps = []


class SelfTestDialog(QDialog):
    def __init__(self, parent):
        super().__init__(parent)
        self.setWindowTitle("Self-test")
        self.resize(760, 420)
        lay = QVBoxLayout(self)
        self.table = QTableWidget(0, 3)
        self.table.setHorizontalHeaderLabels(["Test", "Status", "Detail"])
        self.table.horizontalHeader().setSectionResizeMode(2, QHeaderView.Stretch)
        lay.addWidget(self.table)
        row = QHBoxLayout()
        self.btn_run = QPushButton("Run device self-test")
        self.btn_light = QPushButton("Run light on/off check (2 s)")
        self.btn_close = QPushButton("Close")
        row.addWidget(self.btn_run)
        row.addWidget(self.btn_light)
        row.addStretch()
        row.addWidget(self.btn_close)
        lay.addLayout(row)
        self.btn_close.clicked.connect(self.accept)
        self.summary = QLabel("")
        lay.addWidget(self.summary)

    def show_results(self, results):
        self.table.setRowCount(0)
        counts = {}
        for r in results:
            self.add_row(r["name"], r["status"], r.get("detail", ""))
            counts[r["status"]] = counts.get(r["status"], 0) + 1
        self.summary.setText("  ".join("%s: %d" % kv for kv in sorted(counts.items())))

    def add_row(self, name, status, detail):
        i = self.table.rowCount()
        self.table.insertRow(i)
        self.table.setItem(i, 0, QTableWidgetItem(name))
        it = QTableWidgetItem(status.upper())
        it.setForeground(QColor("white"))
        it.setBackground(QColor(STATUS_COLORS.get(status, "#9e9e9e")))
        self.table.setItem(i, 1, it)
        self.table.setItem(i, 2, QTableWidgetItem(detail))


class MainWindow(QMainWindow):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("Photoacoustic detector")
        self.resize(1400, 860)
        self.settings = QSettings("JII", "pa-detector")
        self.cal = pa_limits.Calibration()
        self.dev = PADevice()
        self.fw = None
        self.valid_freqs = pa_limits.valid_frequencies()

        # measurement state
        self.t_hist = deque()
        self.z_hist = deque()               # complex lock-in values (fraction of FS)
        self.amp_hist = deque()
        self.ph_hist = deque()
        self.z_recent = deque(maxlen=600)
        self.z_dark = None
        self.dark_freq = None
        self.last_data = None
        self.last_burst = None
        self.noise_db = None                # spectral noise floor near f_mod (dBFS)
        self.snr_db = None
        self.seq_expected = None
        self.sweep = None
        self.sweep_result = []
        self.sweep_collect = []
        self.sweep_freq_backup = None
        self.check_seq = None
        self.check_vals = {"off": [], "on": []}
        self.log_file = None
        self.log_writer = None
        self.pending_note = ""
        self.id_timer = QTimer(self)
        self.id_timer.setSingleShot(True)
        self.id_timer.timeout.connect(self.on_no_id)

        self._build_ui()
        self._connect_device_signals()
        self.refresh_ports()

        self.port_timer = QTimer(self)
        self.port_timer.timeout.connect(self.refresh_ports)
        self.port_timer.start(2000)
        self.hb_timer = QTimer(self)
        self.hb_timer.timeout.connect(lambda: self.dev.send("ping"))
        self.hb_timer.start(1000)
        self.load_config(self.settings.value("last_config", None), quiet=True)
        self.update_limits()

    # ------------------------------------------------------------------ UI --
    def _build_ui(self):
        central = QWidget()
        self.setCentralWidget(central)
        root = QVBoxLayout(central)

        # top bar
        top = QHBoxLayout()
        top.addWidget(QLabel("Port:"))
        self.port_combo = QComboBox()
        self.port_combo.setMinimumWidth(260)
        top.addWidget(self.port_combo)
        self.btn_refresh = QPushButton("Refresh")
        self.btn_refresh.clicked.connect(self.refresh_ports)
        top.addWidget(self.btn_refresh)
        self.btn_connect = QPushButton("Connect")
        self.btn_connect.clicked.connect(self.toggle_connect)
        top.addWidget(self.btn_connect)
        top.addSpacing(20)
        self.led_pico = led_label("Pico")
        self.led_fw = led_label("Firmware")
        self.led_adc = led_label("ADC")
        self.led_clk = led_label("Clock")
        self.led_dac = led_label("Pulser")
        self.led_ovl = led_label("Level")
        for w in (self.led_pico, self.led_fw, self.led_adc, self.led_clk, self.led_dac, self.led_ovl):
            top.addWidget(w)
        top.addStretch()
        self.light_state = QLabel("LIGHT OFF")
        self.light_state.setStyleSheet("font-weight:bold;font-size:16px;color:#2e7d32")
        top.addWidget(self.light_state)
        self.btn_kill = QPushButton("LIGHT OFF")
        self.btn_kill.setStyleSheet("QPushButton{background:#c62828;color:white;font-weight:bold;padding:8px 18px;font-size:14px}")
        self.btn_kill.clicked.connect(lambda: self.set_light(False))
        top.addWidget(self.btn_kill)
        root.addLayout(top)

        split = QSplitter(Qt.Horizontal)
        root.addWidget(split, 1)

        # left controls
        scroll = QScrollArea()
        scroll.setWidgetResizable(True)
        left = QWidget()
        scroll.setWidget(left)
        scroll.setMinimumWidth(360)
        scroll.setMaximumWidth(420)
        lv = QVBoxLayout(left)
        lv.addWidget(self._group_excitation())
        lv.addWidget(self._group_adc())
        lv.addWidget(self._group_measurement())
        lv.addWidget(self._group_sweep())
        lv.addWidget(self._group_calibration())
        lv.addWidget(self._group_logging())
        lv.addWidget(self._group_tools())
        lv.addStretch()
        split.addWidget(scroll)

        # right plots
        right = QWidget()
        rv = QVBoxLayout(right)
        self.tabs = QTabWidget()
        rv.addWidget(self.tabs, 1)
        self.tabs.addTab(self._tab_lockin(), "Lock-in")
        self.tabs.addTab(self._tab_spectrum(), "Spectrum")
        self.tabs.addTab(self._tab_time(), "Time domain")
        self.tabs.addTab(self._tab_sweep(), "Sweep")
        self.console = QPlainTextEdit()
        self.console.setReadOnly(True)
        self.console.setMaximumHeight(150)
        self.console.setFont(QFont("Consolas", 9))
        rv.addWidget(self.console)
        split.addWidget(right)
        split.setStretchFactor(1, 1)

        self.statusBar().showMessage("Disconnected")

    def _group_excitation(self):
        g = QGroupBox("Excitation (actopulser)")
        f = QFormLayout(g)
        self.freq_combo = QComboBox()
        for fr in self.valid_freqs:
            self.freq_combo.addItem("%d Hz" % fr, fr)
        self.freq_combo.setCurrentIndex(self.freq_combo.findData(1000))
        self.freq_combo.currentIndexChanged.connect(self.on_freq_changed)
        f.addRow("Modulation", self.freq_combo)
        self.width_spin = QDoubleSpinBox()
        self.width_spin.setRange(pa_limits.MIN_WIDTH_US, pa_limits.MAX_WIDTH_US)
        self.width_spin.setDecimals(1)
        self.width_spin.setSingleStep(0.5)
        self.width_spin.setValue(10.0)
        self.width_spin.setSuffix(" us")
        self.width_spin.valueChanged.connect(self.on_width_changed)
        f.addRow("Pulse width", self.width_spin)
        self.current_spin = QDoubleSpinBox()
        self.current_spin.setRange(0, pa_limits.LED_MAX_MA)
        self.current_spin.setDecimals(1)
        self.current_spin.setSingleStep(10)
        self.current_spin.setValue(500)
        self.current_spin.setSuffix(" mA")
        self.current_spin.valueChanged.connect(self.on_current_changed)
        f.addRow("LED current", self.current_spin)
        self.channel_combo = QComboBox()
        for n in CHANNEL_NAMES:
            self.channel_combo.addItem(n)
        self.channel_combo.currentIndexChanged.connect(self.on_channel_changed)
        f.addRow("Channel", self.channel_combo)
        self.pd_combo = QComboBox()
        for v in pa_limits.PD_VOLTAGES:
            self.pd_combo.addItem("%d V" % v, v)
        self.pd_combo.setCurrentIndex(0)
        self.pd_combo.currentIndexChanged.connect(self.update_limits)
        f.addRow("USB-PD supply (DIP)", self.pd_combo)
        self.info_label = QLabel("")
        self.info_label.setWordWrap(True)
        f.addRow(self.info_label)
        self.warn_label = QLabel("")
        self.warn_label.setWordWrap(True)
        self.warn_label.setStyleSheet("color:#ef6c00")
        f.addRow(self.warn_label)
        self.btn_light = QPushButton("Light ON")
        self.btn_light.setCheckable(True)
        self.btn_light.setStyleSheet("QPushButton:checked{background:#f9a825;font-weight:bold}")
        self.btn_light.clicked.connect(lambda checked: self.set_light(checked))
        f.addRow(self.btn_light)
        return g

    def _group_adc(self):
        g = QGroupBox("ADC (TLV320ADC6120)")
        f = QFormLayout(g)
        self.gain_spin = QDoubleSpinBox()
        self.gain_spin.setRange(0, 42)
        self.gain_spin.setDecimals(1)
        self.gain_spin.setSingleStep(0.5)
        self.gain_spin.setValue(12)
        self.gain_spin.setSuffix(" dB")
        self.gain_spin.valueChanged.connect(self.on_gain_changed)
        f.addRow("Analog gain", self.gain_spin)
        self.btn_autorange = QPushButton("Auto-range gain (peak -> -12 dBFS)")
        self.btn_autorange.clicked.connect(self.autorange)
        f.addRow(self.btn_autorange)
        self.level_label = QLabel("RMS: --   peak: --")
        f.addRow(self.level_label)
        return g

    def _group_measurement(self):
        g = QGroupBox("Measurement")
        f = QFormLayout(g)
        self.avg_spin = QSpinBox()
        self.avg_spin.setRange(1, 600)
        self.avg_spin.setValue(10)
        self.avg_spin.setSuffix(" blocks (x100 ms)")
        f.addRow("Averaging", self.avg_spin)
        self.units_combo = QComboBox()
        self.units_combo.addItems(["dBFS", "mPa (peak)"])
        f.addRow("Amplitude units", self.units_combo)
        row = QHBoxLayout()
        self.btn_dark = QPushButton("Take dark")
        self.btn_dark.setToolTip("Average the current lock-in vector (light off or beam blocked) and subtract it")
        self.btn_dark.clicked.connect(self.take_dark)
        self.btn_dark_clear = QPushButton("Clear")
        self.btn_dark_clear.clicked.connect(self.clear_dark)
        self.chk_dark = QCheckBox("Subtract")
        row.addWidget(self.btn_dark)
        row.addWidget(self.btn_dark_clear)
        row.addWidget(self.chk_dark)
        f.addRow("Dark reference", row)
        self.dark_label = QLabel("none")
        f.addRow(self.dark_label)
        return g

    def _group_sweep(self):
        g = QGroupBox("Frequency sweep (find cell resonance)")
        f = QFormLayout(g)
        self.sweep_from = QComboBox()
        self.sweep_to = QComboBox()
        for fr in self.valid_freqs:
            self.sweep_from.addItem("%d Hz" % fr, fr)
            self.sweep_to.addItem("%d Hz" % fr, fr)
        self.sweep_from.setCurrentIndex(self.sweep_from.findData(200))
        self.sweep_to.setCurrentIndex(self.sweep_to.findData(6000))
        f.addRow("From", self.sweep_from)
        f.addRow("To", self.sweep_to)
        self.sweep_settle = QSpinBox()
        self.sweep_settle.setRange(1, 50)
        self.sweep_settle.setValue(3)
        self.sweep_settle.setSuffix(" blocks settle")
        f.addRow("Settle", self.sweep_settle)
        self.sweep_blocks = QSpinBox()
        self.sweep_blocks.setRange(1, 200)
        self.sweep_blocks.setValue(10)
        self.sweep_blocks.setSuffix(" blocks / step")
        f.addRow("Average", self.sweep_blocks)
        self.btn_sweep = QPushButton("Run sweep")
        self.btn_sweep.clicked.connect(self.toggle_sweep)
        f.addRow(self.btn_sweep)
        self.sweep_label = QLabel("")
        f.addRow(self.sweep_label)
        return g

    def _group_calibration(self):
        g = QGroupBox("Calibration")
        f = QFormLayout(g)
        self.mic_sens = QDoubleSpinBox()
        self.mic_sens.setRange(-60, 0)
        self.mic_sens.setDecimals(1)
        self.mic_sens.setValue(pa_limits.MIC_SENS_DBV_PA)
        self.mic_sens.setSuffix(" dBV/Pa")
        self.mic_sens.valueChanged.connect(lambda v: setattr(self.cal, "mic_sens_dbv_pa", v))
        f.addRow("Mic sensitivity", self.mic_sens)
        self.adc_fs = QDoubleSpinBox()
        self.adc_fs.setRange(0.5, 3.0)
        self.adc_fs.setDecimals(2)
        self.adc_fs.setValue(pa_limits.ADC_FULL_SCALE_VRMS)
        self.adc_fs.setSuffix(" Vrms FS @0dB")
        self.adc_fs.valueChanged.connect(lambda v: setattr(self.cal, "adc_fs_vrms", v))
        f.addRow("ADC full scale", self.adc_fs)
        return g

    def _group_logging(self):
        g = QGroupBox("Logging")
        f = QFormLayout(g)
        row = QHBoxLayout()
        self.log_path = QLineEdit(os.path.join(os.getcwd(), "pa_log_%s.csv" % time.strftime("%Y%m%d_%H%M%S")))
        self.btn_browse = QPushButton("...")
        self.btn_browse.setMaximumWidth(30)
        self.btn_browse.clicked.connect(self.browse_log)
        row.addWidget(self.log_path)
        row.addWidget(self.btn_browse)
        f.addRow("File", row)
        self.btn_log = QPushButton("Start logging")
        self.btn_log.setCheckable(True)
        self.btn_log.clicked.connect(self.toggle_log)
        f.addRow(self.btn_log)
        row2 = QHBoxLayout()
        self.note_edit = QLineEdit()
        self.note_edit.setPlaceholderText("note for the next logged row")
        self.btn_note = QPushButton("Add")
        self.btn_note.clicked.connect(self.add_note)
        row2.addWidget(self.note_edit)
        row2.addWidget(self.btn_note)
        f.addRow("Note", row2)
        return g

    def _group_tools(self):
        g = QGroupBox("Tools")
        v = QVBoxLayout(g)
        self.btn_selftest = QPushButton("Self-test...")
        self.btn_selftest.clicked.connect(self.open_selftest)
        v.addWidget(self.btn_selftest)
        row = QHBoxLayout()
        self.btn_save_cfg = QPushButton("Save config")
        self.btn_save_cfg.clicked.connect(self.save_config_dialog)
        self.btn_load_cfg = QPushButton("Load config")
        self.btn_load_cfg.clicked.connect(self.load_config_dialog)
        row.addWidget(self.btn_save_cfg)
        row.addWidget(self.btn_load_cfg)
        v.addLayout(row)
        self.btn_clear = QPushButton("Clear history")
        self.btn_clear.clicked.connect(self.clear_history)
        v.addWidget(self.btn_clear)
        return g

    def _tab_lockin(self):
        w = QWidget()
        v = QVBoxLayout(w)
        ro = QHBoxLayout()
        self.big_amp = QLabel("--")
        self.big_ph = QLabel("--")
        self.big_snr = QLabel("--")
        for lab, title in ((self.big_amp, "Amplitude"), (self.big_ph, "Phase"), (self.big_snr, "SNR (10 Hz bin)")):
            box = QVBoxLayout()
            t = QLabel(title)
            t.setAlignment(Qt.AlignCenter)
            lab.setAlignment(Qt.AlignCenter)
            lab.setFont(QFont("Segoe UI", 22, QFont.Bold))
            box.addWidget(t)
            box.addWidget(lab)
            ro.addLayout(box)
        v.addLayout(ro)
        self.plot_amp = pg.PlotWidget(title="Lock-in amplitude")
        self.plot_amp.setLabel("bottom", "time", units="s")
        self.plot_amp.showGrid(x=True, y=True, alpha=0.3)
        self.curve_amp_raw = self.plot_amp.plot(pen=pg.mkPen("#90caf9", width=1))
        self.curve_amp = self.plot_amp.plot(pen=pg.mkPen("#1565c0", width=2))
        v.addWidget(self.plot_amp)
        self.plot_ph = pg.PlotWidget(title="Lock-in phase (deg)")
        self.plot_ph.setLabel("bottom", "time", units="s")
        self.plot_ph.showGrid(x=True, y=True, alpha=0.3)
        self.plot_ph.setYRange(-180, 180)
        self.curve_ph_raw = self.plot_ph.plot(pen=pg.mkPen("#ffcc80", width=1))
        self.curve_ph = self.plot_ph.plot(pen=pg.mkPen("#e65100", width=2))
        v.addWidget(self.plot_ph)
        return w

    def _tab_spectrum(self):
        w = QWidget()
        v = QVBoxLayout(w)
        row = QHBoxLayout()
        self.chk_zoom = QCheckBox("Zoom around modulation frequency")
        self.chk_zoom.stateChanged.connect(lambda _: self.redraw_spectrum())
        self.spec_avg = QSpinBox()
        self.spec_avg.setRange(1, 50)
        self.spec_avg.setValue(4)
        self.spec_avg.setPrefix("average ")
        self.spec_avg.setSuffix(" bursts")
        row.addWidget(self.chk_zoom)
        row.addWidget(self.spec_avg)
        row.addStretch()
        self.spec_label = QLabel("")
        row.addWidget(self.spec_label)
        v.addLayout(row)
        self.plot_spec = pg.PlotWidget(title="Spectrum of channel 1 (Hann, 10 Hz bins)")
        self.plot_spec.setLabel("bottom", "frequency", units="Hz")
        self.plot_spec.setLabel("left", "dBFS")
        self.plot_spec.showGrid(x=True, y=True, alpha=0.3)
        self.curve_spec = self.plot_spec.plot(pen=pg.mkPen("#1565c0", width=1))
        self.spec_markers = []
        v.addWidget(self.plot_spec)
        self.spec_hist = deque(maxlen=50)
        return w

    def _tab_time(self):
        w = QWidget()
        v = QVBoxLayout(w)
        self.plot_raw = pg.PlotWidget(title="Raw block (100 ms)")
        self.plot_raw.setLabel("bottom", "time", units="s")
        self.plot_raw.setLabel("left", "fraction of full scale")
        self.plot_raw.showGrid(x=True, y=True, alpha=0.3)
        self.curve_raw = self.plot_raw.plot(pen=pg.mkPen("#1565c0", width=1))
        v.addWidget(self.plot_raw)
        self.plot_sync = pg.PlotWidget(title="Pulse-synchronous average (one modulation period, offset arbitrary but constant)")
        self.plot_sync.setLabel("bottom", "time in period", units="s")
        self.plot_sync.setLabel("left", "fraction of full scale")
        self.plot_sync.showGrid(x=True, y=True, alpha=0.3)
        self.curve_sync = self.plot_sync.plot(pen=pg.mkPen("#e65100", width=2))
        v.addWidget(self.plot_sync)
        return w

    def _tab_sweep(self):
        w = QWidget()
        v = QVBoxLayout(w)
        self.plot_sweep = pg.PlotWidget(title="Amplitude vs modulation frequency")
        self.plot_sweep.setLabel("bottom", "frequency", units="Hz")
        self.plot_sweep.setLabel("left", "dBFS")
        self.plot_sweep.showGrid(x=True, y=True, alpha=0.3)
        self.curve_sweep = self.plot_sweep.plot(pen=pg.mkPen("#1565c0", width=2), symbol="o", symbolSize=6)
        v.addWidget(self.plot_sweep)
        self.plot_sweep_ph = pg.PlotWidget(title="Phase vs modulation frequency")
        self.plot_sweep_ph.setLabel("bottom", "frequency", units="Hz")
        self.plot_sweep_ph.setLabel("left", "deg")
        self.plot_sweep_ph.showGrid(x=True, y=True, alpha=0.3)
        self.curve_sweep_ph = self.plot_sweep_ph.plot(pen=pg.mkPen("#e65100", width=2), symbol="o", symbolSize=6)
        v.addWidget(self.plot_sweep_ph)
        row = QHBoxLayout()
        self.btn_sweep_export = QPushButton("Export sweep CSV")
        self.btn_sweep_export.clicked.connect(self.export_sweep)
        row.addWidget(self.btn_sweep_export)
        row.addStretch()
        v.addLayout(row)
        return w

    # ------------------------------------------------------- device signals --
    def _connect_device_signals(self):
        d = self.dev
        d.ident.connect(self.on_ident)
        d.data.connect(self.on_data)
        d.burst.connect(self.on_burst)
        d.status.connect(self.on_status)
        d.selftest.connect(self.on_selftest)
        d.ack.connect(self.on_ack)
        d.error.connect(lambda m: self.log("ERROR  " + m, "#c62828"))
        d.log.connect(lambda m: self.log("%s  %s" % (m.get("level", "").upper(), m.get("msg", "")),
                                         "#ef6c00" if m.get("level") in ("warn", "error") else None))
        d.raw.connect(lambda s: self.log("RAW    " + s, "#757575"))
        d.connected.connect(self.on_connected)

    def log(self, text, color=None):
        ts = time.strftime("%H:%M:%S")
        if color:
            self.console.appendHtml('<span style="color:%s">%s  %s</span>' % (color, ts, text))
        else:
            self.console.appendPlainText("%s  %s" % (ts, text))

    # ----------------------------------------------------------- connection --
    def refresh_ports(self):
        ports = list_ports()
        current = self.port_combo.currentData()
        items = [(p[0], "%s  %s%s" % (p[0], p[1], "  [Pico, MicroPython]" if p[3] else ("  [Pico]" if p[2] else "")))
                 for p in ports]
        if [i[0] for i in items] != [self.port_combo.itemData(i) for i in range(self.port_combo.count())]:
            self.port_combo.blockSignals(True)
            self.port_combo.clear()
            for dev, text in items:
                self.port_combo.addItem(text, dev)
            idx = self.port_combo.findData(current)
            self.port_combo.setCurrentIndex(idx if idx >= 0 else 0)
            self.port_combo.blockSignals(False)
        picos = [p for p in ports if p[2]]
        set_led(self.led_pico, "pass" if picos else "fail")
        boot = find_bootsel_drive()
        if boot and not self.dev.is_open():
            self.statusBar().showMessage("Pico in BOOTSEL mode on %s: copy a MicroPython UF2 there, then upload firmware/*.py" % boot)
        elif not self.dev.is_open():
            self.statusBar().showMessage("Disconnected. %d serial port(s), %d Pico(s)" % (len(ports), len(picos)))

    def toggle_connect(self):
        if self.dev.is_open():
            self.set_light(False)
            self.dev.close()
            return
        port = self.port_combo.currentData()
        if not port:
            QMessageBox.warning(self, "No port", "No serial port selected.")
            return
        try:
            self.dev.open(port)
        except Exception as e:
            QMessageBox.critical(self, "Connection failed", str(e))
            return
        self.fw = None
        set_led(self.led_fw, "?")
        self.id_timer.start(3000)
        self.log("connected to %s, waiting for firmware id..." % port)

    def on_connected(self, ok):
        self.btn_connect.setText("Disconnect" if ok else "Connect")
        if not ok:
            for lab in (self.led_fw, self.led_adc, self.led_clk, self.led_dac, self.led_ovl):
                set_led(lab, "?")
            self.fw = None
            self.show_light(False)
            self.statusBar().showMessage("Disconnected")
            if self.sweep:
                self.abort_sweep("disconnected")

    def on_no_id(self):
        if self.dev.is_open() and self.fw is None:
            self.log("no firmware answer: is main.py running? (Ctrl-D in a REPL, or copy firmware/*.py)", "#c62828")
            set_led(self.led_fw, "fail")

    def on_ident(self, msg):
        self.fw = msg
        self.id_timer.stop()
        set_led(self.led_fw, "pass")
        set_led(self.led_adc, "pass" if msg.get("adc") else "fail")
        set_led(self.led_dac, "pass" if msg.get("dac") else "fail")
        self.log("firmware %s %s  adc=%s pulser=%s" % (msg.get("fw"), msg.get("version"), msg.get("adc"), msg.get("dac")))
        self.statusBar().showMessage("Connected: %s v%s on %s" % (msg.get("fw"), msg.get("version"), self.dev.port))
        self.push_all_settings()
        self.dev.send("status")

    def push_all_settings(self):
        """Send the GUI state to a freshly connected board (light stays off)."""
        d = self.dev
        d.send("light off")
        d.send("set heartbeat 5")
        d.send("set burst 10")
        d.send("set channel %d" % self.channel_combo.currentIndex())
        d.send("set freq %d" % self.freq_combo.currentData())
        d.send("set width %.1f" % self.width_spin.value())
        d.send("set gain %.1f" % self.gain_spin.value())
        d.send("set current %.1f" % self.current_spin.value())
        d.send("start")

    def on_status(self, msg):
        adc = msg.get("adc", {})
        set_led(self.led_clk, "pass" if adc.get("clock_ok") else "fail")
        self.show_light(bool(msg.get("light")))
        self.log("status: mode=%s fs=%s ratio=%s light=%s %.0f mA ch%d gain %.1f dB" % (
            adc.get("mode"), adc.get("fs"), adc.get("bclk_ratio"), msg.get("light"),
            msg.get("current_ma", 0), msg.get("channel", 0), msg.get("gain_db", 0)))

    def on_ack(self, msg):
        k, v = msg.get("key"), msg.get("value")
        if k == "light":
            self.show_light(bool(v))
        elif k == "current" and isinstance(v, (int, float)) and abs(v - self.current_spin.value()) > 0.3:
            self.current_spin.blockSignals(True)
            self.current_spin.setValue(v)
            self.current_spin.blockSignals(False)

    # -------------------------------------------------------------- controls --
    def show_light(self, on):
        self.btn_light.blockSignals(True)
        self.btn_light.setChecked(on)
        self.btn_light.blockSignals(False)
        self.light_state.setText("LIGHT ON" if on else "LIGHT OFF")
        self.light_state.setStyleSheet("font-weight:bold;font-size:16px;color:%s" % ("#c62828" if on else "#2e7d32"))

    def set_light(self, on):
        if on:
            ok, warns, _ = self.current_settings_check()
            if not ok:
                QMessageBox.warning(self, "Setting refused", "\n".join(warns))
                self.show_light(False)
                return
            if not (self.fw and self.fw.get("dac")):
                QMessageBox.warning(self, "No pulser", "The actopulser DAC was not detected; cannot switch the light on.")
                self.show_light(False)
                return
        self.dev.send("light on" if on else "light off")
        if not self.dev.is_open():
            self.show_light(False)

    def current_settings_check(self):
        return pa_limits.check_settings(self.freq_combo.currentData(), self.width_spin.value(),
                                        self.current_spin.value(), self.pd_combo.currentData())

    def update_limits(self, *_):
        ok, warns, info = self.current_settings_check()
        self.info_label.setText(
            "duty %.2f %%   avg LED %.1f mA / %.2f W   driver %.2f W avg (%.1f W peak)   limit %.0f mA" % (
                info["duty"] * 100, info["avg_led_ma"], info["avg_led_w"], info["mosfet_avg_w"],
                info["mosfet_peak_w"], info["allowed_ma"]))
        self.warn_label.setText("\n".join(warns))
        self.warn_label.setStyleSheet("color:%s" % ("#c62828" if not ok else "#ef6c00"))
        self.current_spin.setMaximum(info["allowed_ma"])

    def on_freq_changed(self, *_):
        self.update_limits()
        if self.dev.is_open():
            self.dev.send("set freq %d" % self.freq_combo.currentData())
        if self.z_dark is not None and self.dark_freq != self.freq_combo.currentData():
            self.clear_dark()
            self.log("dark reference cleared (frequency changed)", "#ef6c00")
        self.clear_history()

    def on_width_changed(self, *_):
        self.update_limits()
        if self.dev.is_open():
            self.dev.send("set width %.1f" % self.width_spin.value())

    def on_current_changed(self, *_):
        self.update_limits()
        if self.dev.is_open():
            self.dev.send("set current %.1f" % self.current_spin.value())

    def on_channel_changed(self, idx):
        if self.dev.is_open():
            self.dev.send("set channel %d" % idx)

    def on_gain_changed(self, *_):
        if self.dev.is_open():
            self.dev.send("set gain %.1f" % self.gain_spin.value())

    def autorange(self):
        if not self.last_data:
            return
        peak_db = pa_limits.dbfs(self.last_data["peak"] / 32768.0)
        new = self.gain_spin.value() + (-12.0 - peak_db)
        new = max(0.0, min(42.0, round(new * 2) / 2))
        self.gain_spin.setValue(new)
        self.log("auto-range: peak %.1f dBFS -> gain %.1f dB" % (peak_db, new))

    # -------------------------------------------------------------- data ------
    def on_data(self, msg):
        self.last_data = msg
        if self.seq_expected is not None and msg["seq"] != self.seq_expected:
            self.log("missed %d block(s)" % (msg["seq"] - self.seq_expected), "#ef6c00")
        self.seq_expected = msg["seq"] + 1
        set_led(self.led_clk, "pass" if msg.get("clk") else "fail")
        set_led(self.led_ovl, "fail" if msg.get("ovl") else "pass")
        self.show_light(bool(msg.get("light")))
        self.level_label.setText("RMS: %.1f dBFS   peak: %.1f dBFS" % (
            msg["rms_db"], pa_limits.dbfs(msg["peak"] / 32768.0)))

        z = msg["amp"] * np.exp(1j * math.radians(msg["ph"]))
        if self.chk_dark.isChecked() and self.z_dark is not None:
            z = z - self.z_dark
        t = msg["t"] / 1000.0
        self.t_hist.append(t)
        self.z_hist.append(z)
        self.z_recent.append(z)
        n = self.avg_spin.value()
        zs = list(self.z_hist)[-n:]
        z_avg = sum(zs) / len(zs)
        self.amp_hist.append(abs(z_avg))
        self.ph_hist.append(math.degrees(np.angle(z_avg)))
        while self.t_hist and self.t_hist[-1] - self.t_hist[0] > HISTORY_S:
            self.t_hist.popleft()
            self.z_hist.popleft()
            self.amp_hist.popleft()
            self.ph_hist.popleft()

        # readouts
        gain = msg.get("gain", self.gain_spin.value())
        amp_db = pa_limits.dbfs(abs(z_avg))
        self.big_amp.setText(self.fmt_amp(abs(z_avg), gain))
        self.big_ph.setText("%+.1f deg" % math.degrees(np.angle(z_avg)))
        if self.noise_db is not None:
            self.snr_db = amp_db - self.noise_db
            self.big_snr.setText("%.1f dB" % self.snr_db)

        # plots
        tt = np.fromiter(self.t_hist, float)
        zz = np.fromiter(self.z_hist, complex)
        if self.units_combo.currentIndex() == 1:
            conv = lambda a: self.cal.amp_fs_to_pa(a, gain) * 1e3
            self.plot_amp.setLabel("left", "mPa peak")
        else:
            conv = lambda a: 20 * np.log10(np.maximum(a, 1e-12))
            self.plot_amp.setLabel("left", "dBFS")
        self.curve_amp_raw.setData(tt, conv(np.abs(zz)))
        self.curve_amp.setData(tt, conv(np.fromiter(self.amp_hist, float)))
        self.curve_ph_raw.setData(tt, np.degrees(np.angle(zz)))
        self.curve_ph.setData(tt, np.fromiter(self.ph_hist, float))

        # sequences and logging
        if self.sweep:
            self.sweep.feed(msg)
        if self.check_seq:
            self.check_seq.feed(msg)
        self.write_log_row(msg, z, z_avg)

    def fmt_amp(self, amp_fs, gain):
        if self.units_combo.currentIndex() == 1:
            pa = self.cal.amp_fs_to_pa(amp_fs, gain)
            return "%.3f mPa" % (pa * 1e3) if pa < 1 else "%.3f Pa" % pa
        return "%.1f dBFS" % pa_limits.dbfs(amp_fs)

    def on_burst(self, samples, meta):
        x = samples.astype(np.float64) / 32768.0
        n = len(x)
        fs = meta.get("fs", 48000)
        self.last_burst = (x, meta)
        # time domain
        t = np.arange(n) / fs
        self.curve_raw.setData(t, x)
        period = int(meta.get("period", 0) or 0)
        if period and n % period == 0:
            folded = x.reshape(n // period, period).mean(axis=0)
            self.curve_sync.setData(np.arange(period) / fs, folded)
        # spectrum
        win = np.hanning(n)
        spec = np.fft.rfft(x * win)
        mag = 2.0 * np.abs(spec) / win.sum()
        db = 20 * np.log10(np.maximum(mag, 1e-12))
        self.spec_hist.append(db)
        self.spec_freqs = np.fft.rfftfreq(n, 1.0 / fs)
        self.redraw_spectrum()

    def redraw_spectrum(self):
        if not self.spec_hist:
            return
        k = min(self.spec_avg.value(), len(self.spec_hist))
        db = np.mean(list(self.spec_hist)[-k:], axis=0)
        f = self.spec_freqs
        self.curve_spec.setData(f, db)
        fmod = self.freq_combo.currentData()
        for m in self.spec_markers:
            self.plot_spec.removeItem(m)
        self.spec_markers = []
        for h in range(1, 8):
            fh = fmod * h
            if fh > f[-1]:
                break
            line = pg.InfiniteLine(fh, angle=90, pen=pg.mkPen("#c62828" if h == 1 else "#ef9a9a", style=Qt.DashLine))
            self.plot_spec.addItem(line)
            self.spec_markers.append(line)
        # noise estimate: median of bins within +-300 Hz of f_mod, excluding +-2 bins around harmonics
        df = f[1] - f[0]
        sel = (np.abs(f - fmod) <= 300) & (f > 20)
        for h in range(1, 8):
            sel &= np.abs(f - fmod * h) > 2.5 * df
        if sel.any():
            self.noise_db = float(np.median(db[sel]))
            ibin = int(round(fmod / df))
            peak_db = float(db[ibin]) if ibin < len(db) else float("nan")
            self.spec_label.setText("bin at %d Hz: %.1f dBFS   noise near f_mod: %.1f dBFS   SNR %.1f dB" % (
                fmod, peak_db, self.noise_db, peak_db - self.noise_db))
        if self.chk_zoom.isChecked():
            self.plot_spec.setXRange(max(0, fmod - 500), fmod + 500)
        else:
            self.plot_spec.setXRange(0, f[-1])

    # -------------------------------------------------------------- dark ------
    def take_dark(self):
        n = self.avg_spin.value()
        zs = list(self.z_recent)[-n:]
        if len(zs) < 1:
            return
        # z_recent holds already-corrected values; recompute uncorrected
        if self.chk_dark.isChecked() and self.z_dark is not None:
            zs = [z + self.z_dark for z in zs]
        self.z_dark = sum(zs) / len(zs)
        self.dark_freq = self.freq_combo.currentData()
        self.chk_dark.setChecked(True)
        self.dark_label.setText("dark: %.1f dBFS @ %+.0f deg (%d blocks, %d Hz)" % (
            pa_limits.dbfs(abs(self.z_dark)), math.degrees(np.angle(self.z_dark)), len(zs), self.dark_freq))
        self.log("dark reference taken: " + self.dark_label.text())
        self.clear_history()          # restart the running average with corrected values

    def clear_dark(self):
        self.z_dark = None
        self.dark_freq = None
        self.chk_dark.setChecked(False)
        self.dark_label.setText("none")
        self.clear_history()

    def clear_history(self):
        self.t_hist.clear()
        self.z_hist.clear()
        self.amp_hist.clear()
        self.ph_hist.clear()
        self.seq_expected = None

    # -------------------------------------------------------------- sweep -----
    def toggle_sweep(self):
        if self.sweep:
            self.abort_sweep("aborted by user")
            return
        if not self.dev.is_open() or not self.fw:
            QMessageBox.warning(self, "Not connected", "Connect to the device first.")
            return
        f0, f1 = self.sweep_from.currentData(), self.sweep_to.currentData()
        freqs = [f for f in self.valid_freqs if min(f0, f1) <= f <= max(f0, f1)]
        if len(freqs) < 2:
            QMessageBox.warning(self, "Sweep", "Choose a wider range.")
            return
        # LED limit must hold at every frequency in the range
        for f in freqs:
            ok, warns, _ = pa_limits.check_settings(f, self.width_spin.value(), self.current_spin.value(),
                                                    self.pd_combo.currentData())
            if not ok:
                QMessageBox.warning(self, "Sweep refused", "At %d Hz: %s" % (f, "; ".join(warns)))
                return
        self.sweep_freq_backup = self.freq_combo.currentData()
        self.sweep_result = []
        settle, nblk = self.sweep_settle.value(), self.sweep_blocks.value()
        steps = []
        for f in freqs:
            steps.append((lambda f=f: self._sweep_set_freq(f), settle))
            steps.append((lambda: self.sweep_collect.clear(), nblk))
            steps.append((lambda f=f: self._sweep_store(f), 0))
        self.sweep = Sequence(steps, self._sweep_done, collect=lambda m: self.sweep_collect.append(
            m["amp"] * np.exp(1j * math.radians(m["ph"]))))
        self.btn_sweep.setText("Abort sweep")
        self.sweep_label.setText("running: %d frequencies" % len(freqs))
        self.tabs.setCurrentIndex(3)
        self.log("sweep started: %d Hz .. %d Hz, %d steps" % (freqs[0], freqs[-1], len(freqs)))
        self.sweep.start()

    def _sweep_set_freq(self, f):
        self.freq_combo.blockSignals(True)
        self.freq_combo.setCurrentIndex(self.freq_combo.findData(f))
        self.freq_combo.blockSignals(False)
        self.dev.send("set freq %d" % f)
        self.sweep_label.setText("measuring %d Hz" % f)

    def _sweep_store(self, f):
        if self.sweep_collect:
            z = sum(self.sweep_collect) / len(self.sweep_collect)
            self.sweep_result.append((f, abs(z), math.degrees(np.angle(z))))
            fr = [r[0] for r in self.sweep_result]
            self.curve_sweep.setData(fr, [pa_limits.dbfs(r[1]) for r in self.sweep_result])
            self.curve_sweep_ph.setData(fr, [r[2] for r in self.sweep_result])

    def _sweep_done(self):
        self.sweep = None
        self.btn_sweep.setText("Run sweep")
        if self.sweep_result:
            best = max(self.sweep_result, key=lambda r: r[1])
            self.sweep_label.setText("done. max %.1f dBFS at %d Hz" % (pa_limits.dbfs(best[1]), best[0]))
            self.log("sweep done: maximum %.1f dBFS at %d Hz" % (pa_limits.dbfs(best[1]), best[0]))
        if self.sweep_freq_backup:
            self.freq_combo.setCurrentIndex(self.freq_combo.findData(self.sweep_freq_backup))

    def abort_sweep(self, why):
        if self.sweep:
            self.sweep.abort()
        self.sweep = None
        self.btn_sweep.setText("Run sweep")
        self.sweep_label.setText("aborted: " + why)
        if self.sweep_freq_backup and self.dev.is_open():
            self.freq_combo.setCurrentIndex(self.freq_combo.findData(self.sweep_freq_backup))

    def export_sweep(self):
        if not self.sweep_result:
            return
        path, _ = QFileDialog.getSaveFileName(self, "Export sweep", "sweep.csv", "CSV (*.csv)")
        if not path:
            return
        with open(path, "w", newline="") as fh:
            w = csv.writer(fh)
            w.writerow(["freq_hz", "amp_fs", "amp_dbfs", "phase_deg", "width_us", "current_ma", "gain_db"])
            for f, a, p in self.sweep_result:
                w.writerow([f, "%.6g" % a, "%.2f" % pa_limits.dbfs(a), "%.2f" % p,
                            self.width_spin.value(), self.current_spin.value(), self.gain_spin.value()])
        self.log("sweep exported to " + path)

    # -------------------------------------------------------------- self-test -
    def open_selftest(self):
        dlg = SelfTestDialog(self)
        dlg.btn_run.clicked.connect(lambda: self.run_selftest(dlg))
        dlg.btn_light.clicked.connect(lambda: self.run_light_check(dlg))
        self._selftest_dlg = dlg
        self.run_selftest(dlg)
        dlg.exec()
        self._selftest_dlg = None

    def run_selftest(self, dlg):
        dlg.table.setRowCount(0)
        dlg.add_row("pc_serial_port", "pass" if self.dev.is_open() else "fail",
                    self.dev.port or "not connected")
        dlg.add_row("pc_firmware_id", "pass" if self.fw else "fail",
                    "%s %s" % (self.fw.get("fw"), self.fw.get("version")) if self.fw else "no id received")
        if self.dev.is_open():
            self.dev.send("selftest")
        else:
            dlg.summary.setText("connect to run the device tests")

    def on_selftest(self, results):
        dlg = getattr(self, "_selftest_dlg", None)
        if dlg is None:
            return
        for r in results:
            dlg.add_row(r["name"], r["status"], r.get("detail", ""))
        bad = [r["name"] for r in results if r["status"] == "fail"]
        dlg.summary.setText("device tests: %d, failed: %s" % (len(results), ", ".join(bad) if bad else "none"))
        self.log("self-test: %d results, failed: %s" % (len(results), ", ".join(bad) if bad else "none"),
                 "#c62828" if bad else "#2e7d32")

    def run_light_check(self, dlg):
        if not self.dev.is_open() or not self.fw:
            return
        ok, warns, _ = self.current_settings_check()
        if not ok:
            QMessageBox.warning(self, "Setting refused", "\n".join(warns))
            return
        if self.current_spin.value() <= 0:
            QMessageBox.warning(self, "Light check", "Set a non-zero LED current first.")
            return
        self.check_vals = {"off": [], "on": []}
        cur = self.check_vals
        state = {"key": "off"}
        steps = [
            (lambda: self.set_light(False), 3),
            (lambda: state.update(key="off"), 10),
            (lambda: self.set_light(True), 5),
            (lambda: state.update(key="on"), 10),
            (lambda: self.set_light(False), 0),
        ]
        self.check_seq = Sequence(steps, lambda: self._light_check_done(dlg),
                                  collect=lambda m: cur[state["key"]].append(m["amp"]))
        dlg.btn_light.setEnabled(False)
        self.check_seq.start()

    def _light_check_done(self, dlg):
        self.check_seq = None
        dlg.btn_light.setEnabled(True)
        off = self.check_vals["off"]
        on = self.check_vals["on"]
        if not off or not on:
            dlg.add_row("light_functional", "fail", "no data collected")
            return
        a_off = pa_limits.dbfs(float(np.mean(off)))
        a_on = pa_limits.dbfs(float(np.mean(on)))
        gain = a_on - a_off
        status = "pass" if gain > 6 else ("warn" if gain > 1 else "fail")
        detail = "light off %.1f dBFS, light on %.1f dBFS, difference %.1f dB at %d Hz, %.0f mA" % (
            a_off, a_on, gain, self.freq_combo.currentData(), self.current_spin.value())
        dlg.add_row("light_functional", status, detail)
        self.log("light check: " + detail, STATUS_COLORS[status])

    # -------------------------------------------------------------- logging ---
    def browse_log(self):
        path, _ = QFileDialog.getSaveFileName(self, "Log file", self.log_path.text(), "CSV (*.csv)")
        if path:
            self.log_path.setText(path)

    def toggle_log(self, checked):
        if checked:
            try:
                new = not os.path.exists(self.log_path.text()) or os.path.getsize(self.log_path.text()) == 0
                self.log_file = open(self.log_path.text(), "a", newline="")
                self.log_writer = csv.writer(self.log_file)
                if new:
                    self.log_writer.writerow([
                        "time_iso", "seq", "freq_hz", "width_us", "current_ma", "channel", "gain_db", "light",
                        "i", "q", "amp_fs", "amp_dbfs", "amp_mpa_peak", "phase_deg",
                        "amp_avg_fs", "amp_avg_dbfs", "phase_avg_deg", "avg_blocks", "dark_subtracted",
                        "rms_dbfs", "peak", "clock_ok", "overload", "noise_dbfs", "snr_db", "note"])
                self.btn_log.setText("Stop logging")
                self.log("logging to " + self.log_path.text())
            except OSError as e:
                QMessageBox.critical(self, "Log file", str(e))
                self.btn_log.setChecked(False)
        else:
            if self.log_file:
                self.log_file.close()
            self.log_file = None
            self.log_writer = None
            self.btn_log.setText("Start logging")
            self.log("logging stopped")

    def add_note(self):
        self.pending_note = self.note_edit.text().strip()
        self.note_edit.clear()
        if self.pending_note:
            self.log("note queued: " + self.pending_note)

    def write_log_row(self, msg, z, z_avg):
        if not self.log_writer:
            return
        gain = msg.get("gain", self.gain_spin.value())
        self.log_writer.writerow([
            time.strftime("%Y-%m-%dT%H:%M:%S"), msg["seq"], msg.get("freq"), self.width_spin.value(),
            msg.get("ma"), self.channel_combo.currentIndex(), gain, int(bool(msg.get("light"))),
            msg["i"], msg["q"], "%.6g" % abs(z), "%.2f" % pa_limits.dbfs(abs(z)),
            "%.5g" % (self.cal.amp_fs_to_pa(abs(z), gain) * 1e3), "%.2f" % math.degrees(np.angle(z)),
            "%.6g" % abs(z_avg), "%.2f" % pa_limits.dbfs(abs(z_avg)), "%.2f" % math.degrees(np.angle(z_avg)),
            self.avg_spin.value(), int(self.chk_dark.isChecked() and self.z_dark is not None),
            "%.2f" % msg["rms_db"], msg["peak"], int(bool(msg.get("clk"))), int(bool(msg.get("ovl"))),
            "" if self.noise_db is None else "%.2f" % self.noise_db,
            "" if self.snr_db is None else "%.2f" % self.snr_db, self.pending_note])
        self.pending_note = ""
        self.log_file.flush()

    # -------------------------------------------------------------- config ----
    def config_dict(self):
        return {
            "freq": self.freq_combo.currentData(), "width_us": self.width_spin.value(),
            "current_ma": self.current_spin.value(), "channel": self.channel_combo.currentIndex(),
            "pd_volts": self.pd_combo.currentData(), "gain_db": self.gain_spin.value(),
            "avg_blocks": self.avg_spin.value(), "units": self.units_combo.currentIndex(),
            "mic_sens_dbv_pa": self.mic_sens.value(), "adc_fs_vrms": self.adc_fs.value(),
            "sweep_from": self.sweep_from.currentData(), "sweep_to": self.sweep_to.currentData(),
            "sweep_settle": self.sweep_settle.value(), "sweep_blocks": self.sweep_blocks.value(),
        }

    def apply_config(self, c):
        def combo_data(combo, val):
            i = combo.findData(val)
            if i >= 0:
                combo.setCurrentIndex(i)
        combo_data(self.freq_combo, c.get("freq", 1000))
        self.width_spin.setValue(c.get("width_us", 10.0))
        combo_data(self.pd_combo, c.get("pd_volts", 5))
        self.current_spin.setValue(c.get("current_ma", 500))
        self.channel_combo.setCurrentIndex(c.get("channel", 0))
        self.gain_spin.setValue(c.get("gain_db", 12))
        self.avg_spin.setValue(c.get("avg_blocks", 10))
        self.units_combo.setCurrentIndex(c.get("units", 0))
        self.mic_sens.setValue(c.get("mic_sens_dbv_pa", pa_limits.MIC_SENS_DBV_PA))
        self.adc_fs.setValue(c.get("adc_fs_vrms", pa_limits.ADC_FULL_SCALE_VRMS))
        combo_data(self.sweep_from, c.get("sweep_from", 200))
        combo_data(self.sweep_to, c.get("sweep_to", 6000))
        self.sweep_settle.setValue(c.get("sweep_settle", 3))
        self.sweep_blocks.setValue(c.get("sweep_blocks", 10))

    def save_config_dialog(self):
        path, _ = QFileDialog.getSaveFileName(self, "Save config", "pa_config.json", "JSON (*.json)")
        if path:
            with open(path, "w") as fh:
                json.dump(self.config_dict(), fh, indent=2)
            self.settings.setValue("last_config", path)
            self.log("config saved to " + path)

    def load_config_dialog(self):
        path, _ = QFileDialog.getOpenFileName(self, "Load config", "", "JSON (*.json)")
        if path:
            self.load_config(path)

    def load_config(self, path, quiet=False):
        if not path or not os.path.exists(path):
            return
        try:
            with open(path) as fh:
                self.apply_config(json.load(fh))
            self.settings.setValue("last_config", path)
            if not quiet:
                self.log("config loaded from " + path)
        except (OSError, ValueError) as e:
            if not quiet:
                QMessageBox.warning(self, "Config", str(e))

    # -------------------------------------------------------------- shutdown --
    def closeEvent(self, ev):
        try:
            self.set_light(False)
            time.sleep(0.1)
            self.dev.close()
            if self.log_file:
                self.log_file.close()
        except Exception:
            pass
        ev.accept()


def main():
    app = QApplication(sys.argv)
    win = MainWindow()
    win.show()
    sys.exit(app.exec())


if __name__ == "__main__":
    main()

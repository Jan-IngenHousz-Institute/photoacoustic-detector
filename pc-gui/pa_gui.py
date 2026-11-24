"""
pa_gui.py - desktop GUI for the photoacoustic detector (Pico + TLV320ADC6120 shield
+ LED current source on GP0).

    python pa_gui.py            the detector on a USB serial port
    python pa_gui.py --sim      also offers a simulated detector ("SIM", pa_sim.py), no hardware needed

Features: Pico discovery (incl. BOOTSEL detection), firmware handshake, layered
self-test, LED modulation 200 Hz - 10 kHz, FFT of the microphone stream (up to
5 s long, power averaged) with harmonic markers and SNR, lock-in amplitude/phase
strip charts with averaging and dark subtraction, time-domain and
pulse-synchronous views, frequency sweep, recorded sweeps (one .txt with the
metadata, the raw blocks and samples and the analysed amplitude and phase, in
the photoacoustic-data folder, see pa_record.py), CSV logging with full
settings context, config save/load, heartbeat safety.
"""

import csv
import json
import math
import os
import platform
import sys
import time
from collections import deque

try:
    import numpy as np
    import pyqtgraph as pg
    from PySide6.QtCore import Qt, QTimer, QSettings, QUrl
    from PySide6.QtGui import QFont, QColor, QDesktopServices
    from PySide6.QtWidgets import (
        QApplication, QMainWindow, QWidget, QLabel, QPushButton, QComboBox,
        QDoubleSpinBox, QSpinBox, QCheckBox, QGroupBox, QFormLayout, QVBoxLayout,
        QHBoxLayout, QTabWidget, QPlainTextEdit, QFileDialog, QMessageBox, QDialog,
        QTableWidget, QTableWidgetItem, QLineEdit, QSplitter, QScrollArea, QHeaderView,
        QSizePolicy, QInputDialog,
    )
except ImportError as _e:
    _conda = "conda" in sys.prefix.lower() or "conda" in getattr(sys, "base_prefix", "").lower()
    (sys.stderr or sys.stdout or open(os.devnull, "w")).write(
        "\nCannot start the GUI: %s\n\n" % _e
        + ("This is an Anaconda/conda Python: its MSVC runtime DLLs are older than the ones PySide6 "
           "needs, so Qt cannot load here.\n" if _conda else
           "A required package is missing or broken in this Python (%s).\n" % sys.executable)
        + "Start the GUI with pc-gui\\run_gui.bat (Windows: creates its own environment, installs "
          "Python if needed), or use the standalone build from the GitHub releases page, or run\n"
          "    py -3.13 pc-gui\\pa_gui.py\n"
          "with a python.org interpreter after `pip install -r pc-gui/requirements.txt`. See README.md.\n")
    sys.exit(1)

import pa_limits
import pa_record
import pa_sim
from pa_device import PADevice, list_ports, find_bootsel_drive

pg.setConfigOptions(antialias=False, background="w", foreground="k")

GUI_VERSION = "1.1.0"
HISTORY_S = 120
RECORD_WRITE_DELAY_MS = 500         # the last block's raw samples follow its data line
BURST_EVERY = 1                     # raw block every 100 ms: gap-free stream for the FFT
FFT_LENGTHS_S = (0.1, 0.5, 1.0, 2.0, 5.0)
FFT_MAX_AVG = 64
FOLD_BINS = 128                     # phase bins of the pulse-synchronous average
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
    """Runs a list of (callable, blocks_to_wait[, accept]) steps driven by incoming data blocks. With
    accept(msg), only the blocks it accepts count (and are collected): a block measured before a setting
    took effect, but received after the command was sent, is skipped."""

    def __init__(self, steps, on_done, collect=None):
        self.steps = list(steps)
        self.on_done = on_done
        self.collect = collect          # callable(msg) storing values while waiting
        self.remaining = 0
        self.accept = None
        self.active = False

    def start(self):
        self.active = True
        self._advance()

    def _advance(self):
        if not self.steps:
            self.active = False
            self.on_done()
            return
        step = self.steps.pop(0)
        fn, n = step[:2]
        self.accept = step[2] if len(step) > 2 else None
        fn()
        self.remaining = n

    def feed(self, msg):
        if not self.active:
            return
        if self.accept is not None and self.remaining > 0 and not self.accept(msg):
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
        self.setWindowTitle("Photoacoustic detector  -  GUI %s" % GUI_VERSION)
        self.resize(1400, 860)
        self.settings = QSettings("JII", "pa-detector")
        self.data_dir, data_dir_moved = pa_record.data_folder()
        self.cal = pa_limits.Calibration()
        self.dev = PADevice()
        self.fw = None
        self.p_bclk = None                  # modulation period in I2S bit clocks
        self.freq_actual = None             # modulation frequency the firmware makes
        self.light_on = False

        # measurement state
        self.t_hist = deque()
        self.z_hist = deque()               # complex lock-in values (fraction of FS)
        self.amp_hist = deque()
        self.ph_hist = deque()
        self.z_recent = deque(maxlen=600)
        self.z_dark = None
        self.dark_p = None
        self.last_data = None
        self.last_burst = None
        self.noise_db = None                # spectral noise floor near f_mod (dBFS)
        self.snr_db = None
        self.seq_expected = None
        self.rate_ref = None                # (t_ms, seq) for the real-time check
        self.fft_buf = np.zeros(0)          # contiguous samples not yet used by a frame
        self.fft_seq = None
        self.fft_frames = deque(maxlen=FFT_MAX_AVG)   # power spectra, float32
        self.fft_n = None
        self.fft_fs = pa_limits.FS
        self.fft_win = None
        self.spec_freqs = None
        self.spec_db = None
        self.fold_p = None
        self.fold_sum = np.zeros(FOLD_BINS)
        self.fold_cnt = np.zeros(FOLD_BINS)
        self.sweep = None
        self.sweep_result = []
        self.sweep_collect = []
        self.sweep_freq_backup = None
        self.sweep_stale = 0                # blocks still at the old frequency after a sweep step
        self.record = None                  # pa_record.SweepRecord of the sweep being recorded
        self.last_status = None
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
        self.log("GUI %s, data folder %s" % (GUI_VERSION, self.data_dir))
        if data_dir_moved:
            self.log("the launch folder cannot be written: recorded sweeps go to %s" % self.data_dir, "#ef6c00")
        self.refresh_ports()
        self.set_mod_freq(*pa_limits.snap_frequency(self.freq_spin.value()))

        self.port_timer = QTimer(self)
        self.port_timer.timeout.connect(self.refresh_ports)
        self.port_timer.start(2000)
        self.hb_timer = QTimer(self)
        self.hb_timer.timeout.connect(lambda: self.dev.send("ping"))
        self.hb_timer.start(1000)
        self.load_config(self.settings.value("last_config", None), quiet=True)

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
        self.led_mod = led_label("Modulator")
        self.led_ovl = led_label("Level")
        for w in (self.led_pico, self.led_fw, self.led_adc, self.led_clk, self.led_mod, self.led_ovl):
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
        self.tabs.addTab(self._tab_fft(), "FFT")
        self.tabs.addTab(self._tab_lockin(), "Lock-in")
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

    @staticmethod
    def _freq_spin(value):
        s = QDoubleSpinBox()
        s.setRange(pa_limits.F_MIN, pa_limits.F_MAX)
        s.setDecimals(1)
        s.setSingleStep(10)
        s.setSuffix(" Hz")
        s.setValue(value)
        return s

    def _group_excitation(self):
        g = QGroupBox("Excitation (LED current source on GP0)")
        f = QFormLayout(g)
        self.freq_spin = self._freq_spin(1000)
        self.freq_spin.setKeyboardTracking(False)     # send on Enter / arrows, not per keystroke
        self.freq_spin.setToolTip("Square wave, 50 % duty. The period is a whole number of I2S bit "
                                  "clocks (64 per sample), so the frequency snaps to 3.072 MHz / P.")
        self.freq_spin.valueChanged.connect(self.on_freq_changed)
        f.addRow("Modulation", self.freq_spin)
        self.freq_label = QLabel("")
        f.addRow(self.freq_label)
        self.btn_light = QPushButton("Modulation ON")
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
        self.sweep_from = self._freq_spin(200)
        self.sweep_to = self._freq_spin(6000)
        f.addRow("From", self.sweep_from)
        f.addRow("To", self.sweep_to)
        row = QHBoxLayout()
        self.sweep_points = QSpinBox()
        self.sweep_points.setRange(2, 500)
        self.sweep_points.setValue(50)
        self.sweep_points.setSuffix(" points")
        self.chk_sweep_log = QCheckBox("log spacing")
        self.chk_sweep_log.setChecked(True)
        row.addWidget(self.sweep_points)
        row.addWidget(self.chk_sweep_log)
        f.addRow("Steps", row)
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
        row = QHBoxLayout()
        self.btn_sweep = QPushButton("Run sweep")
        self.btn_sweep.clicked.connect(self.toggle_sweep)
        self.btn_record = QPushButton("Record sweep...")
        self.btn_record.setStyleSheet("QPushButton{font-weight:bold}")
        self.btn_record.setToolTip(
            "Asks for a name, runs the sweep and saves <name>.txt in the data folder: a header with the "
            "settings and the device information, the analysed amplitude and phase per frequency, every "
            "averaged 100 ms block and its raw microphone samples")
        self.btn_record.clicked.connect(self.record_sweep)
        row.addWidget(self.btn_sweep)
        row.addWidget(self.btn_record)
        f.addRow(row)
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
        g = QGroupBox("Data and logging")
        f = QFormLayout(g)
        row = QHBoxLayout()
        self.data_dir_label = QLabel(self.data_dir)
        self.data_dir_label.setWordWrap(True)
        self.data_dir_label.setToolTip("Recorded sweeps, logs and exports are saved here")
        self.btn_open_data = QPushButton("Open")
        self.btn_open_data.setMaximumWidth(50)
        self.btn_open_data.clicked.connect(lambda: QDesktopServices.openUrl(QUrl.fromLocalFile(self.data_dir)))
        row.addWidget(self.data_dir_label, 1)
        row.addWidget(self.btn_open_data)
        f.addRow("Data folder", row)
        row = QHBoxLayout()
        self.log_path = QLineEdit(os.path.join(self.data_dir, "pa_log_%s.csv" % time.strftime("%Y%m%d_%H%M%S")))
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
        self.btn_update_fw = QPushButton("Update firmware...")
        self.btn_update_fw.setToolTip("Reboot the Pico into its UF2 bootloader (RPI-RP2 drive)")
        self.btn_update_fw.clicked.connect(self.update_firmware)
        v.addWidget(self.btn_update_fw)
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
        for lab, title in ((self.big_amp, "Amplitude"), (self.big_ph, "Phase"), (self.big_snr, "SNR (vs FFT bin noise)")):
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

    def _tab_fft(self):
        w = QWidget()
        v = QVBoxLayout(w)
        row = QHBoxLayout()
        row.addWidget(QLabel("Length"))
        self.fft_len = QComboBox()
        for s in FFT_LENGTHS_S:
            self.fft_len.addItem("%g s  (%g Hz bins)" % (s, 1 / s), s)
        self.fft_len.setCurrentIndex(FFT_LENGTHS_S.index(1.0))
        self.fft_len.currentIndexChanged.connect(lambda _: self.reset_fft())
        row.addWidget(self.fft_len)
        self.fft_avg = QSpinBox()
        self.fft_avg.setRange(1, FFT_MAX_AVG)
        self.fft_avg.setValue(4)
        self.fft_avg.setPrefix("average ")
        self.fft_avg.setSuffix(" frames")
        self.fft_avg.setToolTip("Power average of Hann-windowed frames with 50 % overlap")
        self.fft_avg.valueChanged.connect(lambda _: self.redraw_spectrum())
        row.addWidget(self.fft_avg)
        self.chk_logx = QCheckBox("Log frequency")
        self.chk_logx.stateChanged.connect(lambda _: self.redraw_spectrum(rescale=True))
        row.addWidget(self.chk_logx)
        self.chk_zoom = QCheckBox("Zoom around f_mod")
        self.chk_zoom.stateChanged.connect(lambda _: self.redraw_spectrum(rescale=True))
        row.addWidget(self.chk_zoom)
        row.addStretch()
        self.btn_fft_export = QPushButton("Export CSV")
        self.btn_fft_export.clicked.connect(self.export_spectrum)
        row.addWidget(self.btn_fft_export)
        v.addLayout(row)
        self.spec_label = QLabel("waiting for samples")
        self.spec_label.setToolTip("f_mod generally falls between FFT bins, so the peak bin can read up to "
                                   "1.4 dB low (Hann scalloping). The Lock-in tab gives the exact amplitude.")
        v.addWidget(self.spec_label)
        self.plot_spec = pg.PlotWidget(title="FFT of the microphone (ADC channel 1, Hann window)")
        self.plot_spec.setLabel("bottom", "frequency", units="Hz")
        self.plot_spec.setLabel("left", "dBFS (sine peak)")
        self.plot_spec.showGrid(x=True, y=True, alpha=0.3)
        self.plot_spec.setDownsampling(auto=True, mode="peak")
        self.plot_spec.setClipToView(True)
        self.plot_spec.disableAutoRange()               # fixed axes: 0..20 kHz, -150..0 dBFS (mouse zoom still works)
        self.curve_spec = self.plot_spec.plot(pen=pg.mkPen("#1565c0", width=1))
        self.set_spec_range(logx=False)
        self.spec_markers = []
        v.addWidget(self.plot_spec)
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
        self.plot_sync = pg.PlotWidget(title="Modulation-synchronous average (one period, offset arbitrary but constant)")
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
        items = [(p[0], "%s  %s%s" % (p[0], p[1], "  [pa-detector]" if p[3] else (
            "  [Pico, MicroPython: old firmware]" if p[4] else ("  [Pico]" if p[2] else ""))))
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
            self.statusBar().showMessage("Pico in BOOTSEL mode on %s: copy pa_detector-<version>.uf2 "
                                         "(firmware/release/ or the standalone zip) onto that drive" % boot)
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
        self.id_retries = 0
        set_led(self.led_fw, "?")
        self.id_timer.start(3000)
        self.log("connected to %s, waiting for firmware id..." % port)

    def on_connected(self, ok):
        self.btn_connect.setText("Disconnect" if ok else "Connect")
        if not ok:
            for lab in (self.led_fw, self.led_adc, self.led_clk, self.led_mod, self.led_ovl):
                set_led(lab, "?")
            self.fw = None
            self.show_light(False)
            self.statusBar().showMessage("Disconnected")
            if self.sweep:
                self.abort_sweep("disconnected")

    def on_no_id(self):
        if not self.dev.is_open() or self.fw is not None:
            return
        self.id_retries = getattr(self, "id_retries", 0) + 1
        if self.id_retries <= 3:            # the first reply can be lost in a torn line from an old session
            self.dev.send("id")
            self.id_timer.start(2000)
            return
        self.log("no firmware answer: flash pa_detector-<version>.uf2 (Tools > Update firmware..., or hold "
                 "BOOTSEL while plugging in and copy the UF2 to the RPI-RP2 drive; see README)", "#c62828")
        set_led(self.led_fw, "fail")

    def on_ident(self, msg):
        self.fw = msg
        self.id_timer.stop()
        set_led(self.led_fw, "pass")
        set_led(self.led_adc, "pass" if msg.get("adc") else "fail")
        vccs = msg.get("driver") == "vccs"
        set_led(self.led_mod, "pass" if vccs else "fail")
        self.log("firmware %s %s  adc=%s driver=%s on GP%s" % (
            msg.get("fw"), msg.get("version"), msg.get("adc"), msg.get("driver"), msg.get("mod_pin")))
        if not vccs:
            self.log("this firmware does not drive the current source on GP0: flash "
                     "firmware/build/pa_detector.uf2 (see README)", "#c62828")
        self.statusBar().showMessage("Connected: %s v%s on %s" % (msg.get("fw"), msg.get("version"), self.dev.port))
        self.push_all_settings()
        self.dev.send("status")

    def push_all_settings(self):
        """Send the GUI state to a freshly connected board (light stays off)."""
        d = self.dev
        d.send("light off")
        d.send("set heartbeat 5")
        d.send("set burst %d" % BURST_EVERY)
        d.send("set freq %.4f" % self.freq_spin.value())
        d.send("set gain %.1f" % self.gain_spin.value())
        d.send("start")
        self.rate_ref = None
        self.reset_fft()

    def on_status(self, msg):
        self.last_status = msg
        if self.record is not None and not self.record.written:
            self.record.status_msgs.append(msg)
        adc = msg.get("adc", {})
        set_led(self.led_clk, "pass" if adc.get("clock_ok") else "fail")
        self.show_light(bool(msg.get("light")))
        extra = ""
        if "mod_sync" in msg:           # Pico SDK firmware (proto 2)
            extra = "  sync=%s overruns=%s skipped=%s dropped bursts=%s" % (
                msg.get("mod_sync"), msg.get("overruns"), msg.get("blocks_skipped"), msg.get("tx_drop_burst"))
        self.log("status: mode=%s fs=%s ratio=%s light=%s %.3f Hz (P=%s) gain %.1f dB%s" % (
            adc.get("mode"), adc.get("fs"), adc.get("bclk_ratio"), msg.get("light"),
            msg.get("freq", 0), msg.get("p_bclk"), msg.get("gain_db", 0), extra))

    def on_ack(self, msg):
        k, v = msg.get("key"), msg.get("value")
        if k == "light":
            self.show_light(bool(v))
        elif k == "freq" and isinstance(v, (int, float)) and msg.get("p_bclk"):
            self.set_mod_freq(msg["p_bclk"], v)

    # -------------------------------------------------------------- controls --
    def show_light(self, on):
        if on != self.light_on:
            self.light_on = on
            self.reset_fft()              # keep light-on and light-off spectra apart
            self.reset_fold()
        self.btn_light.blockSignals(True)
        self.btn_light.setChecked(on)
        self.btn_light.blockSignals(False)
        self.light_state.setText("LIGHT ON" if on else "LIGHT OFF")
        self.light_state.setStyleSheet("font-weight:bold;font-size:16px;color:%s" % ("#c62828" if on else "#2e7d32"))

    def set_light(self, on):
        if on and not (self.dev.is_open() and self.fw):
            QMessageBox.warning(self, "Not connected", "Connect to the device first.")
            self.show_light(False)
            return
        self.dev.send("light on" if on else "light off")
        if not self.dev.is_open():
            self.show_light(False)

    def set_mod_freq(self, p, f_actual):
        """Track the modulation the firmware makes: P bit clocks per period."""
        if p == self.p_bclk:
            return
        self.p_bclk, self.freq_actual = p, f_actual
        self.freq_label.setText("actual %.3f Hz  (P = %d bit clocks)" % (f_actual, p))
        self.reset_fft()
        self.reset_fold()

    def on_freq_changed(self, *_):
        self.set_mod_freq(*pa_limits.snap_frequency(self.freq_spin.value()))
        if self.dev.is_open():
            self.dev.send("set freq %.4f" % self.freq_spin.value())
        if self.z_dark is not None and self.dark_p != self.p_bclk:
            self.clear_dark()
            self.log("dark reference cleared (frequency changed)", "#ef6c00")
        self.clear_history()

    def on_gain_changed(self, *_):
        if self.dev.is_open():
            self.dev.send("set gain %.1f" % self.gain_spin.value())
        self.reset_fft()

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
        self.check_realtime(msg)
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

    def check_realtime(self, msg):
        """The Pico counts the blocks it processed. If it falls behind real time, the I2S
        buffer overflows and samples are lost without a gap in seq: blocks then arrive
        more than 100 ms apart on the Pico clock."""
        if self.rate_ref is None or msg["seq"] < self.rate_ref[1]:
            self.rate_ref = (msg["t"], msg["seq"])
            return
        t0, s0 = self.rate_ref
        if msg["seq"] - s0 >= 50:
            ms_per_block = (msg["t"] - t0) / (msg["seq"] - s0)
            if ms_per_block > 1000.0 * pa_limits.BLOCK / pa_limits.FS * 1.02:
                self.log("Pico slower than real time (%.0f ms per 100 ms block): samples are being "
                         "dropped, FFT frames are not contiguous" % ms_per_block, "#c62828")
            self.rate_ref = (msg["t"], msg["seq"])

    def on_burst(self, samples, meta):
        if self.record is not None:
            self.record.add_burst(samples, meta)
        x = samples.astype(np.float64) / 32768.0
        fs = meta.get("fs", pa_limits.FS)
        self.last_burst = (x, meta)
        self.curve_raw.setData(np.arange(len(x)) / fs, x)
        self.fold_burst(x, meta, fs)
        self.feed_fft(x, meta.get("seq"), fs)

    # -------------------------------------------------------------- FFT -------
    def reset_fft(self):
        self.fft_buf = np.zeros(0)
        self.fft_seq = None
        self.fft_frames.clear()

    def feed_fft(self, x, seq, fs):
        """Append one block to the contiguous stream and cut Hann frames (50 % overlap)."""
        if self.fft_seq is None or seq != self.fft_seq + 1:
            self.fft_buf = np.zeros(0)        # frames must not straddle a gap
        self.fft_seq = seq
        n = int(round(self.fft_len.currentData() * fs))
        if n != self.fft_n or fs != self.fft_fs:
            self.fft_n, self.fft_fs = n, fs
            self.fft_win = np.hanning(n)
            self.fft_frames.clear()
        self.fft_buf = np.concatenate((self.fft_buf, x))
        new = False
        while len(self.fft_buf) >= n:
            spec = np.fft.rfft(self.fft_buf[:n] * self.fft_win)
            self.fft_frames.append((spec.real ** 2 + spec.imag ** 2).astype(np.float32))
            self.fft_buf = self.fft_buf[n // 2:]
            new = True
        if new:
            self.redraw_spectrum()
        elif not self.fft_frames:
            self.spec_label.setText("collecting %.1f of %.1f s" % (len(self.fft_buf) / fs, n / fs))

    def redraw_spectrum(self, rescale=False):
        if not self.fft_frames:
            return
        k = min(self.fft_avg.value(), len(self.fft_frames))
        power = np.mean(np.array(list(self.fft_frames)[-k:]), axis=0)
        mag = 2.0 * np.sqrt(power) / self.fft_win.sum()      # sine peak, 1.0 = full scale
        db = 20 * np.log10(np.maximum(mag, 1e-12))
        f = np.fft.rfftfreq(self.fft_n, 1.0 / self.fft_fs)
        self.spec_freqs, self.spec_db = f, db
        logx = self.chk_logx.isChecked()
        self.plot_spec.setLogMode(x=logx, y=False)
        self.curve_spec.setData(f[1:], db[1:])
        xpos = (lambda v: math.log10(v)) if logx else (lambda v: v)
        fmod = self.freq_actual
        for m in self.spec_markers:
            self.plot_spec.removeItem(m)
        self.spec_markers = []
        for h in range(1, 8):
            fh = fmod * h
            if fh > f[-1]:
                break
            line = pg.InfiniteLine(xpos(fh), angle=90, pen=pg.mkPen("#c62828" if h == 1 else "#ef9a9a", style=Qt.DashLine))
            self.plot_spec.addItem(line)
            self.spec_markers.append(line)
        # noise estimate: median of bins within +-300 Hz of f_mod, excluding +-2 bins around harmonics
        df = f[1] - f[0]
        sel = (np.abs(f - fmod) <= 300) & (f > 20)
        for h in range(1, 8):
            sel &= np.abs(f - fmod * h) > 2.5 * df
        if sel.any():
            self.noise_db = float(np.median(db[sel]))
            # f_mod falls between bins in general: take the largest of the nearest three
            ibin = int(round(fmod / df))
            peak_db = float(db[max(0, ibin - 1):ibin + 2].max())
            gain = self.last_data.get("gain", self.gain_spin.value()) if self.last_data else self.gain_spin.value()
            self.spec_label.setText(
                "f_mod %.2f Hz, peak bin: %s (%.1f dBFS)   noise near f_mod: %.1f dBFS/bin   SNR %.1f dB   "
                "bins %.2f Hz, %d of %d frames averaged" % (
                    fmod, self.fmt_amp(10 ** (peak_db / 20), gain), peak_db, self.noise_db,
                    peak_db - self.noise_db, df, k, self.fft_avg.value()))
        if self.chk_zoom.isChecked():
            self.plot_spec.setXRange(xpos(max(f[1], fmod - 500)), xpos(fmod + 500), padding=0)
        elif rescale:
            self.set_spec_range(logx)

    def set_spec_range(self, logx):
        """Default spectrum axes: 0 .. 20 kHz (20 Hz .. 20 kHz on a log axis), -150 .. 0 dBFS."""
        if logx:
            self.plot_spec.setXRange(math.log10(20.0), math.log10(20000.0), padding=0)
        else:
            self.plot_spec.setXRange(0.0, 20000.0, padding=0)
        self.plot_spec.setYRange(-150.0, 0.0, padding=0)

    def export_spectrum(self):
        if self.spec_db is None:
            return
        path, _ = QFileDialog.getSaveFileName(self, "Export FFT", os.path.join(self.data_dir, "fft.csv"),
                                              "CSV (*.csv)")
        if not path:
            return
        k = min(self.fft_avg.value(), len(self.fft_frames))
        with open(path, "w", newline="") as fh:
            w = csv.writer(fh)
            w.writerow(["freq_hz", "amp_dbfs", "f_mod_hz", "light", "gain_db", "fft_len_s", "frames_averaged"])
            for f, db in zip(self.spec_freqs, self.spec_db):
                w.writerow(["%.4f" % f, "%.2f" % db, "%.4f" % self.freq_actual, int(self.light_on),
                            self.gain_spin.value(), self.fft_n / self.fft_fs, k])
        self.log("FFT exported to " + path)

    # ------------------------------------------------ synchronous average -----
    def reset_fold(self):
        self.fold_p = None
        self.fold_sum = np.zeros(FOLD_BINS)
        self.fold_cnt = np.zeros(FOLD_BINS)

    def fold_burst(self, x, meta, fs):
        """Average the samples over one modulation period, binned by reference phase."""
        p = meta.get("p_bclk")
        if not p or "ph0" not in meta:
            return
        if p != self.fold_p:
            self.reset_fold()
            self.fold_p = p
        ph = (meta["ph0"] + np.arange(len(x)) * pa_limits.BCLK_PER_FRAME / p) % 1.0
        idx = np.minimum((ph * FOLD_BINS).astype(int), FOLD_BINS - 1)
        self.fold_sum += np.bincount(idx, weights=x, minlength=FOLD_BINS)
        self.fold_cnt += np.bincount(idx, minlength=FOLD_BINS)
        period_s = p / (pa_limits.BCLK_PER_FRAME * fs)
        self.curve_sync.setData((np.arange(FOLD_BINS) + 0.5) / FOLD_BINS * period_s,
                                self.fold_sum / np.maximum(self.fold_cnt, 1))

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
        self.dark_p = self.p_bclk
        self.chk_dark.setChecked(True)
        self.dark_label.setText("dark: %.1f dBFS @ %+.0f deg (%d blocks, %.1f Hz)" % (
            pa_limits.dbfs(abs(self.z_dark)), math.degrees(np.angle(self.z_dark)), len(zs), self.freq_actual))
        self.log("dark reference taken: " + self.dark_label.text())
        self.clear_history()          # restart the running average with corrected values

    def clear_dark(self):
        self.z_dark = None
        self.dark_p = None
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
    def sweep_plan(self):
        log = self.chk_sweep_log.isChecked()
        return pa_limits.sweep_frequencies(self.sweep_from.value(), self.sweep_to.value(),
                                           self.sweep_points.value(), log), log

    def update_sweep_buttons(self):
        self.btn_sweep.setText("Abort sweep" if self.sweep else "Run sweep")
        self.btn_record.setEnabled(self.sweep is None and self.record is None)

    def toggle_sweep(self):
        if self.sweep:
            self.abort_sweep("aborted by user")
            return
        if self.record is None:
            self.start_sweep()

    def record_sweep(self):
        """Ask for a name, then run the sweep and save it as <name>.txt in the data folder."""
        if self.sweep or self.record is not None:
            return
        if not self.dev.is_open() or not self.fw:
            QMessageBox.warning(self, "Not connected", "Connect to the device first.")
            return
        freqs = self.sweep_plan()[0]
        if len(freqs) < 2:
            QMessageBox.warning(self, "Sweep", "Choose a wider range.")
            return
        size_mb = len(freqs) * self.sweep_blocks.value() * pa_limits.BLOCK * 6.2e-6   # ~6 characters per sample
        if size_mb > 200:
            ans = QMessageBox.question(self, "Record sweep",
                                       "%d frequencies x %d blocks: the record will be about %.0f MB of text "
                                       "(raw samples included).\n\nRecord anyway?" % (
                                           len(freqs), self.sweep_blocks.value(), size_mb))
            if ans != QMessageBox.StandardButton.Yes:
                return
        if not self.light_on:
            ans = QMessageBox.question(self, "Record sweep",
                                       "The modulation is OFF: the sweep would record the cell without light "
                                       "(a dark sweep).\n\nRecord anyway?")
            if ans != QMessageBox.StandardButton.Yes:
                return
        target = self.ask_record_name()
        if target:
            self.start_sweep(record=target)

    def ask_record_name(self):
        """(name, path) for a new record, or None if cancelled."""
        text = ""
        while True:
            text, ok = QInputDialog.getText(
                self, "Record sweep", "Name of the record. It is saved as <name>.txt in\n%s" % self.data_dir,
                QLineEdit.EchoMode.Normal, text)
            if not ok:
                return None
            name = pa_record.clean_name(text)
            if not name:
                QMessageBox.warning(self, "Record sweep", "Type a name for the record.")
                continue
            path = os.path.join(self.data_dir, name + ".txt")
            if os.path.exists(path):
                ans = QMessageBox.question(self, "Record sweep",
                                           "%s.txt already exists in the data folder.\n\nReplace it?" % name)
                if ans != QMessageBox.StandardButton.Yes:
                    continue
            return name, path

    def record_metadata(self, freqs, log):
        """Header sections of a record: everything known about the device and the settings at the start."""
        from PySide6 import __version__ as pyside_version
        fw = {k: v for k, v in (self.fw or {}).items() if k != "type"}
        dark = "none"
        if self.z_dark is not None:
            dark = "%.3f dBFS at %+.2f deg, taken at %.4f Hz; not subtracted in this record" % (
                pa_limits.dbfs(abs(self.z_dark)), math.degrees(np.angle(self.z_dark)),
                pa_limits.snap_frequency(self.freq_spin.value())[1] if self.dark_p is None
                else pa_limits.BCLK_PER_FRAME * pa_limits.FS / self.dark_p)
        return [
            ("software", [
                ("gui", "pa_gui %s" % GUI_VERSION), ("standalone_build", bool(getattr(sys, "frozen", False))),
                ("python", platform.python_version()), ("pyside6", pyside_version), ("pyqtgraph", pg.__version__),
                ("numpy", np.__version__), ("os", platform.platform())]),
            ("device", [("port", self.dev.port)] + [("firmware." + k, v) for k, v in fw.items()]),
            ("sweep", [
                ("from_hz", self.sweep_from.value()), ("to_hz", self.sweep_to.value()),
                ("points_requested", self.sweep_points.value()), ("spacing", "log" if log else "linear"),
                ("frequencies_planned", len(freqs)), ("frequencies_hz", ["%.4f" % f for f in freqs]),
                ("settle_blocks", self.sweep_settle.value()), ("average_blocks", self.sweep_blocks.value()),
                ("block_s", pa_limits.BLOCK / pa_limits.FS),
                ("averaging", "vector (complex) mean of the lock-in results of the averaged blocks; the blocks "
                              "seen while a step settled are not recorded")]),
            ("measurement", [
                ("adc_gain_db", self.gain_spin.value()), ("modulation_on_at_start", self.light_on),
                ("modulation_hz_before_sweep", self.freq_actual), ("dark_reference", dark)]),
            ("calibration", [
                ("mic_sensitivity_dbv_pa", self.cal.mic_sens_dbv_pa),
                ("adc_full_scale_vrms_at_0db", self.cal.adc_fs_vrms),
                ("mpa_peak_from_fs", "amp_fs * adc_full_scale_vrms * sqrt(2) / 10^(adc_gain_db / 20) "
                                     "/ 10^(mic_sensitivity_dbv_pa / 20) * 1000")]),
            ("definitions", [
                ("amplitude", "sine peak at the modulation frequency as a fraction of the ADC full scale "
                              "(1 = 0 dBFS = 2 Vrms differential at 0 dB gain): the sweep's gain curve"),
                ("phase", "lag of the microphone signal behind the LED rising edge, as seen at the ADC output; "
                          "for the physical response subtract the ADC group delay, "
                          "360 * f * firmware.adc_latency_samples / firmware.fs degrees, and the high-pass "
                          "filter's lead, atan(firmware.hpf_hz / f) (README, Reading the numbers)")]),
            ("gui settings", self.config_dict()),
        ]

    def start_sweep(self, record=None):
        """Run the sweep; with record = (name, path) also record it."""
        if not self.dev.is_open() or not self.fw:
            QMessageBox.warning(self, "Not connected", "Connect to the device first.")
            return False
        freqs, log = self.sweep_plan()
        if len(freqs) < 2:
            QMessageBox.warning(self, "Sweep", "Choose a wider range.")
            return False
        for p in (self.plot_sweep, self.plot_sweep_ph):
            p.setLogMode(x=log, y=False)
        self.sweep_freq_backup = self.freq_spin.value()
        self.sweep_result = []
        if record:
            self.record = pa_record.SweepRecord(record[0], record[1], self.record_metadata(freqs, log), self.cal)
            self.dev.send("status")             # device status at the start, for the header
        settle, nblk = self.sweep_settle.value(), self.sweep_blocks.value()
        steps = []
        for f in freqs:
            at_f = lambda m, f=f: self._sweep_at_freq(m, f)
            steps.append((lambda f=f: self._sweep_set_freq(f), settle, at_f))
            steps.append((self._sweep_begin_average, nblk, at_f))
            steps.append((lambda f=f: self._sweep_store(f), 0))
        self.sweep = Sequence(steps, self._sweep_done, collect=self._sweep_collect)
        self.update_sweep_buttons()
        self.sweep_label.setText("%s: %d frequencies" % ("recording " + record[0] if record else "running",
                                                         len(freqs)))
        self.tabs.setCurrentIndex(3)
        self.log("sweep %s: %.1f Hz .. %.1f Hz, %d steps" % (
            "recording to " + record[1] if record else "started", freqs[0], freqs[-1], len(freqs)))
        self.sweep.start()
        return True

    def _sweep_set_freq(self, f):
        self.freq_spin.blockSignals(True)
        self.freq_spin.setValue(f)
        self.freq_spin.blockSignals(False)
        p, f_actual = pa_limits.snap_frequency(f)
        self.set_mod_freq(p, f_actual)
        self.sweep_stale = 0
        self.dev.send("set freq %.4f" % f)
        if self.record is not None:
            self.record.new_step(f, f_actual, p)
        self.sweep_label.setText("%s %.1f Hz" % ("recording" if self.record is not None else "measuring", f))

    def _sweep_at_freq(self, m, f):
        """True for a block the firmware made at the step's frequency. Blocks still queued from the previous
        step are skipped; if the device never reports the new frequency the sweep stops."""
        if abs(m.get("freq", f) - pa_limits.snap_frequency(f)[1]) <= 1e-3:
            return True
        self.sweep_stale += 1
        if self.sweep_stale == 50:
            QTimer.singleShot(0, lambda: self.abort_sweep(
                "the device still reports %.1f Hz instead of %.1f Hz" % (m.get("freq", 0.0), f)))
        return False

    def _sweep_begin_average(self):
        """The step has settled: average (and record) the next blocks."""
        self.sweep_collect.clear()
        if self.record is not None:
            self.record.begin_average()

    def _sweep_collect(self, m):
        self.sweep_collect.append(m["amp"] * np.exp(1j * math.radians(m["ph"])))
        if self.record is not None:
            self.record.add_block(m)

    def _sweep_store(self, f):
        if self.sweep_collect:
            z = sum(self.sweep_collect) / len(self.sweep_collect)
            self.sweep_result.append((f, abs(z), math.degrees(np.angle(z))))
            fr = [r[0] for r in self.sweep_result]
            self.curve_sweep.setData(fr, [pa_limits.dbfs(r[1]) for r in self.sweep_result])
            self.curve_sweep_ph.setData(fr, [r[2] for r in self.sweep_result])

    def _sweep_done(self):
        self.sweep = None
        if self.sweep_result:
            best = max(self.sweep_result, key=lambda r: r[1])
            self.sweep_label.setText("done. max %.1f dBFS at %.1f Hz" % (pa_limits.dbfs(best[1]), best[0]))
            self.log("sweep done: maximum %.1f dBFS at %.1f Hz" % (pa_limits.dbfs(best[1]), best[0]))
        if self.record is not None:
            self.finish_record("complete")
        self.update_sweep_buttons()
        if self.sweep_freq_backup:
            self.restore_sweep_freq()

    def restore_sweep_freq(self):
        if self.freq_spin.value() == self.sweep_freq_backup:
            self.on_freq_changed()        # setValue would not emit valueChanged
        else:
            self.freq_spin.setValue(self.sweep_freq_backup)

    def abort_sweep(self, why):
        if self.sweep:
            self.sweep.abort()
        self.sweep = None
        self.sweep_label.setText("aborted: " + why)
        if self.record is not None:
            self.finish_record("aborted: " + why)
        self.update_sweep_buttons()
        if self.sweep_freq_backup and self.dev.is_open():
            self.restore_sweep_freq()

    def finish_record(self, status):
        """End the record and save it a moment later, when the last block's raw samples have arrived."""
        rec = self.record
        if rec is None or rec.finished:
            return
        rec.finish(status)
        if self.dev.is_open():
            self.dev.send("status")             # device status at the end, for the header
        QTimer.singleShot(RECORD_WRITE_DELAY_MS, lambda: self.write_record(rec))

    def write_record(self, rec):
        if rec.written:
            return
        path = rec.path
        try:
            os.makedirs(os.path.dirname(path), exist_ok=True)
            rec.write(path)
        except OSError as e:
            self.log("could not save the record %s: %s" % (path, e), "#c62828")
            path, _ = QFileDialog.getSaveFileName(
                self, "Could not save %s (%s): save the record elsewhere" % (os.path.basename(path), e),
                os.path.join(os.path.expanduser("~"), os.path.basename(path)), "Text (*.txt)")
            try:
                if path:
                    rec.write(path)
            except OSError as e2:
                QMessageBox.critical(self, "Record sweep", "Could not save the record:\n%s" % e2)
        if rec.written:
            blocks = rec.blocks()
            missing = rec.missing_samples()
            self.log("sweep recorded (%s): %s, %d frequencies, %d blocks, %.1f MB" % (
                rec.status, rec.written, len({k for k, _ in blocks}), len(blocks),
                os.path.getsize(rec.written) / 1e6), "#2e7d32" if rec.status == "complete" else "#ef6c00")
            if missing:
                self.log("%d block(s) have no raw samples (dropped on the USB link); the file says which"
                         % len(missing), "#ef6c00")
            self.sweep_label.setText(self.sweep_label.text() + "\nsaved as %s" % os.path.basename(rec.written))
            self.statusBar().showMessage("Sweep recorded: %s" % rec.written, 15000)
        if self.record is rec:
            self.record = None
        self.update_sweep_buttons()

    def export_sweep(self):
        if not self.sweep_result:
            return
        path, _ = QFileDialog.getSaveFileName(self, "Export sweep", os.path.join(self.data_dir, "sweep.csv"),
                                              "CSV (*.csv)")
        if not path:
            return
        with open(path, "w", newline="") as fh:
            w = csv.writer(fh)
            w.writerow(["freq_hz", "amp_fs", "amp_dbfs", "phase_deg", "gain_db"])
            for f, a, p in self.sweep_result:
                w.writerow(["%.4f" % f, "%.6g" % a, "%.2f" % pa_limits.dbfs(a), "%.2f" % p,
                            self.gain_spin.value()])
        self.log("sweep exported to " + path)

    # -------------------------------------------------------------- self-test -
    def update_firmware(self):
        """Reboot the connected Pico into its UF2 bootloader; the user then copies the UF2."""
        how = ("A drive called RPI-RP2 will appear: copy pa_detector-<version>.uf2 (from firmware/release/ "
               "or the standalone zip) onto it. The Pico restarts with the new firmware in a few seconds; "
               "press Refresh and connect again (the port number may change).")
        if not self.dev.is_open():
            QMessageBox.information(self, "Update firmware",
                                    "Connect to the detector first. Without a working firmware, hold the "
                                    "BOOTSEL button on the Pico while plugging the USB cable in.\n\n" + how)
            return
        ans = QMessageBox.question(self, "Update firmware",
                                   "Switch the light off, disconnect and reboot the Pico into its bootloader?\n\n" + how)
        if ans != QMessageBox.StandardButton.Yes:
            return
        self.set_light(False)
        self.dev.send("bootsel")
        self.dev.close()
        self.log("sent bootsel: copy the UF2 onto the RPI-RP2 drive, then press Refresh and connect again")

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
        detail = "light off %.1f dBFS, light on %.1f dBFS, difference %.1f dB at %.1f Hz" % (
            a_off, a_on, gain, self.freq_actual)
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
                        "time_iso", "seq", "freq_hz", "gain_db", "light",
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
            time.strftime("%Y-%m-%dT%H:%M:%S"), msg["seq"], "%.4f" % msg.get("freq", 0.0),
            gain, int(bool(msg.get("light"))),
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
            "freq": self.freq_spin.value(), "gain_db": self.gain_spin.value(),
            "avg_blocks": self.avg_spin.value(), "units": self.units_combo.currentIndex(),
            "mic_sens_dbv_pa": self.mic_sens.value(), "adc_fs_vrms": self.adc_fs.value(),
            "fft_len_s": self.fft_len.currentData(), "fft_avg": self.fft_avg.value(),
            "fft_logx": self.chk_logx.isChecked(),
            "sweep_from": self.sweep_from.value(), "sweep_to": self.sweep_to.value(),
            "sweep_points": self.sweep_points.value(), "sweep_log": self.chk_sweep_log.isChecked(),
            "sweep_settle": self.sweep_settle.value(), "sweep_blocks": self.sweep_blocks.value(),
        }

    def apply_config(self, c):
        def combo_data(combo, val):
            i = combo.findData(val)
            if i >= 0:
                combo.setCurrentIndex(i)
        self.freq_spin.setValue(c.get("freq", 1000))
        self.gain_spin.setValue(c.get("gain_db", 12))
        self.avg_spin.setValue(c.get("avg_blocks", 10))
        self.units_combo.setCurrentIndex(c.get("units", 0))
        self.mic_sens.setValue(c.get("mic_sens_dbv_pa", pa_limits.MIC_SENS_DBV_PA))
        self.adc_fs.setValue(c.get("adc_fs_vrms", pa_limits.ADC_FULL_SCALE_VRMS))
        combo_data(self.fft_len, c.get("fft_len_s", 1.0))
        self.fft_avg.setValue(c.get("fft_avg", 4))
        self.chk_logx.setChecked(c.get("fft_logx", False))
        self.sweep_from.setValue(c.get("sweep_from", 200))
        self.sweep_to.setValue(c.get("sweep_to", 6000))
        self.sweep_points.setValue(c.get("sweep_points", 50))
        self.chk_sweep_log.setChecked(c.get("sweep_log", True))
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
        rec = self.record
        if rec is not None:                     # never lose a recording: save what there is now
            if self.sweep:
                self.sweep.abort()
                self.sweep = None
            rec.finish("aborted: window closed")
            self.write_record(rec)
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
    if "--sim" in sys.argv:
        pa_sim.enable()
        sys.argv = [a for a in sys.argv if a != "--sim"]
    if "--check" in sys.argv:
        # start-up check used by the launchers and the packaged build: build the
        # window off-screen, report the versions and exit
        os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")
        from PySide6 import __version__ as pyside_version
        app = QApplication([a for a in sys.argv if a != "--check"])
        win = MainWindow()
        msg = ("GUI %s check ok: Python %d.%d, PySide6 %s, pyqtgraph %s, numpy %s, %d serial port(s), "
               "data folder %s") % (
            GUI_VERSION, sys.version_info[0], sys.version_info[1], pyside_version, pg.__version__,
            np.__version__, len(list_ports()), win.data_dir)
        if sys.stdout:                  # None in a windowed (PyInstaller) build
            print(msg)
        win.close()
        return
    app = QApplication(sys.argv)
    win = MainWindow()
    win.show()
    sys.exit(app.exec())


if __name__ == "__main__":
    main()

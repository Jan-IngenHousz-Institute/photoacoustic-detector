"""
pa_record.py - recorded frequency sweeps of the photoacoustic detector.

A record is one tab-separated text file, <name>.txt, in the data folder:

    # key: value lines         metadata: record, software, device, settings, calibration, column meanings
    [analysed]                 one row per frequency: amplitude (the sweep's gain curve) and phase
    [blocks]                   one row per 100 ms block that went into the average: the firmware's lock-in values
    [samples]                  one row per block: its raw microphone samples (int16, full scale 32768)

read_record(path) reads it back (numpy arrays per column); see README.md, "Recorded sweeps".

The data folder is "photoacoustic-data" in the folder the program is launched from: the folder of
PhotoacousticDetector.exe for the standalone build, the current folder when run from source. When that
folder cannot be written (for example under Program Files) it falls back to Documents.
"""

import math
import os
import sys
import tempfile
import time

import numpy as np

import pa_limits

FORMAT = "pa-sweep-record 1"
DATA_DIR_NAME = "photoacoustic-data"
SECTIONS = ("analysed", "blocks", "samples")

_BAD_CHARS = '<>:"/\\|?*'
_RESERVED = {"CON", "PRN", "AUX", "NUL"} | {"COM%d" % i for i in range(1, 10)} | {"LPT%d" % i for i in range(1, 10)}

ANALYSED_COLUMNS = [
    ("step", "frequency step, 1 = first"),
    ("f_set_hz", "frequency requested by the sweep plan"),
    ("f_hz", "modulation frequency the firmware made: 3.072 MHz / p_bclk"),
    ("p_bclk", "modulation period in I2S bit clocks"),
    ("n_blocks", "100 ms blocks averaged"),
    ("amp_fs", "amplitude of the vector average of the blocks, sine peak as a fraction of the ADC full scale"),
    ("amp_dbfs", "amp_fs in dBFS"),
    ("amp_rel_db", "amp_dbfs relative to the largest amplitude of this sweep (0 dB at the maximum)"),
    ("amp_mpa_peak", "amp_fs converted to sound pressure, mPa peak, with the calibration above and adc_gain_db"),
    ("phase_deg", "phase of the vector average: lag of the microphone signal behind the LED rising edge"),
    ("amp_sem_fs", "standard error of the vector average (spread of the blocks / sqrt(n)), fraction of full scale"),
    ("phase_sem_deg", "phase uncertainty from amp_sem_fs: degrees(amp_sem_fs / amp_fs)"),
    ("re_fs", "real part of the vector average, fraction of full scale"),
    ("im_fs", "imaginary part of the vector average (amplitude * exp(i * phase))"),
    ("adc_gain_db", "ADC analog gain during the step"),
    ("light", "1 if the modulation was on in every averaged block, 0 if off in every block, -1 if mixed"),
    ("overloads", "averaged blocks flagged as overloaded (|sample| >= 98 % of full scale)"),
    ("freq_mismatch", "averaged blocks whose firmware frequency differs from f_hz (should be 0)"),
]

BLOCK_COLUMNS = [
    ("step", "frequency step the block belongs to"),
    ("seq", "block number from the firmware (100 ms each); pairs the block with its row in [samples]"),
    ("t_ms", "Pico time of the block, ms since the firmware started"),
    ("pc_t_s", "PC time the block arrived, s after the record started"),
    ("fw_freq_hz", "modulation frequency the firmware reported with the block"),
    ("light", "1 = modulation on during the block"),
    ("adc_gain_db", "ADC analog gain the firmware reported with the block"),
    ("i", "lock-in in-phase sum as sent by the firmware (raw units)"),
    ("q", "lock-in quadrature sum as sent by the firmware (raw units)"),
    ("amp_fs", "lock-in amplitude of the block, sine peak as a fraction of full scale (24-bit samples)"),
    ("amp_dbfs", "amp_fs in dBFS"),
    ("phase_deg", "lock-in phase of the block: lag behind the LED rising edge"),
    ("rms_fs", "RMS of the block, fraction of full scale"),
    ("rms_dbfs", "rms_fs in dBFS"),
    ("dc_fs", "mean of the block, fraction of full scale"),
    ("peak", "largest |sample| of the block, int16 units"),
    ("overload", "1 = |sample| >= 98 % of full scale in the block"),
    ("clock_ok", "1 = the ADC reported a valid clock"),
    ("mod_seq", "modulator restart counter from the firmware"),
    ("ph0_cycles", "lock-in reference phase at the first sample of the block, cycles (from the raw-sample message)"),
    ("p_bclk", "modulation period of the block in I2S bit clocks (from the raw-sample message): the reference "
               "advances 64 / p_bclk cycles per sample"),
    ("samples", "1 = the block's raw samples are in [samples], 0 = they were dropped on the USB link"),
]


# ---------------------------------------------------------------- folder and file names
def launch_folder():
    """Folder the program runs from: the .exe's folder for the standalone build, else the current folder."""
    if getattr(sys, "frozen", False):
        return os.path.dirname(os.path.abspath(sys.executable))
    return os.getcwd()


def _writable(folder):
    try:
        os.makedirs(folder, exist_ok=True)
        with tempfile.TemporaryFile(dir=folder):
            pass
        return True
    except OSError:
        return False


def data_folder(base=None):
    """photoacoustic-data in `base` (default: the launch folder), created if needed. Falls back to
    Documents/photoacoustic-data, then to the user's home folder, when the folder cannot be written.
    Returns (path, fell_back)."""
    first = os.path.join(base or launch_folder(), DATA_DIR_NAME)
    if _writable(first):
        return os.path.realpath(first), False
    home = os.path.expanduser("~")
    for alt in (os.path.join(home, "Documents", DATA_DIR_NAME), os.path.join(home, DATA_DIR_NAME)):
        if _writable(alt):
            return os.path.realpath(alt), True
    return first, True


def clean_name(name):
    """A file name from what the user typed: characters Windows forbids become '_', no trailing dots or
    spaces, no '.txt' typed twice, no reserved device names. '' if nothing usable is left."""
    s = "".join("_" if (c in _BAD_CHARS or ord(c) < 32) else c for c in (name or "").strip())
    if s.lower().endswith(".txt"):
        s = s[:-4]
    s = s.strip().rstrip(" .")
    if s and s.split(".")[0].upper() in _RESERVED:
        s = "_" + s
    return s[:120]


# ---------------------------------------------------------------- the record
def _fmt(v):
    if v is None:
        return ""
    if isinstance(v, bool):
        return "1" if v else "0"
    if isinstance(v, float):
        return "" if math.isnan(v) else "%.10g" % v
    if isinstance(v, (list, tuple)):
        return ", ".join(_fmt(x) for x in v)
    return str(v).replace("\r", " ").replace("\n", " ")


def _flatten(d, prefix=""):
    out = []
    for k, v in d.items():
        if isinstance(v, dict):
            out += _flatten(v, prefix + str(k) + ".")
        else:
            out.append((prefix + str(k), v))
    return out


class SweepRecord:
    """Everything of one recorded sweep: the plan, the averaged blocks of every step and their raw samples.

    meta: list of (section title, dict or list of (key, value)) written into the header as given.
    """

    def __init__(self, name, path, meta, cal):
        self.name = name
        self.path = path
        self.meta = list(meta)
        self.cal = cal
        self.t0 = time.time()
        self.started = time.strftime("%Y-%m-%dT%H:%M:%S")
        self.finished = None
        self.status = "running"
        self.steps = []                 # {"f_set", "f_hz", "p_bclk", "blocks": [data messages]}
        self.bursts = {}                # seq -> (int16 samples, burst message without the samples)
        self.status_msgs = []           # device status messages received while recording
        self.written = None             # path once written

    # ---- collecting
    def new_step(self, f_set, f_hz, p_bclk):
        self.steps.append({"f_set": f_set, "f_hz": f_hz, "p_bclk": p_bclk, "blocks": []})

    def begin_average(self):
        """Blocks seen while the step settled do not count; their raw samples are dropped."""
        if self.steps:
            self.steps[-1]["blocks"] = []
        keep = self.wanted_seqs()
        self.bursts = {s: v for s, v in self.bursts.items() if s in keep}

    def add_block(self, msg):
        if self.finished or not self.steps:
            return
        m = dict(msg)
        m["_pc_t"] = time.time() - self.t0
        self.steps[-1]["blocks"].append(m)

    def add_burst(self, samples, meta):
        seq = meta.get("seq")
        if seq is None:
            return
        if self.finished and (seq not in self.wanted_seqs() or seq in self.bursts):
            return
        self.bursts[seq] = (np.array(samples, dtype=np.int16), {k: v for k, v in meta.items() if k != "b64"})

    def finish(self, status):
        if not self.finished:
            self.finished = time.strftime("%Y-%m-%dT%H:%M:%S")
            self.t_end = time.time()
            self.status = status

    # ---- views
    def blocks(self):
        """(step number, data message) of every averaged block, in order."""
        return [(k, b) for k, st in enumerate(self.steps, 1) for b in st["blocks"]]

    def wanted_seqs(self):
        return {b["seq"] for _, b in self.blocks()}

    def missing_samples(self):
        return sorted(s for s in self.wanted_seqs() if s not in self.bursts)

    def analyse(self):
        """One dict per step with averaged blocks, in ANALYSED_COLUMNS order."""
        rows = []
        for k, st in enumerate(self.steps, 1):
            bl = st["blocks"]
            if not bl:
                continue
            z = np.array([b["amp"] * np.exp(1j * math.radians(b["ph"])) for b in bl])
            zm = complex(z.mean())
            n = len(z)
            amp = abs(zm)
            sem = float(np.sqrt(np.sum(np.abs(z - zm) ** 2) / (n * (n - 1)))) if n > 1 else float("nan")
            gain = float(np.mean([b.get("gain", 0.0) for b in bl]))
            lights = {bool(b.get("light")) for b in bl}
            rows.append({
                "step": k, "f_set_hz": st["f_set"], "f_hz": st["f_hz"], "p_bclk": st["p_bclk"], "n_blocks": n,
                "amp_fs": amp, "amp_dbfs": pa_limits.dbfs(amp), "amp_rel_db": None,
                "amp_mpa_peak": self.cal.amp_fs_to_pa(amp, gain) * 1e3,
                "phase_deg": math.degrees(math.atan2(zm.imag, zm.real)),
                "amp_sem_fs": sem, "phase_sem_deg": math.degrees(sem / amp) if amp > 0 and n > 1 else float("nan"),
                "re_fs": zm.real, "im_fs": zm.imag,
                "adc_gain_db": gain,
                "light": (1 if lights == {True} else 0 if lights == {False} else -1),
                "overloads": sum(1 for b in bl if b.get("ovl")),
                "freq_mismatch": sum(1 for b in bl if abs(b.get("freq", st["f_hz"]) - st["f_hz"]) > 1e-3),
            })
        if rows:
            top = max(r["amp_dbfs"] for r in rows)
            for r in rows:
                r["amp_rel_db"] = r["amp_dbfs"] - top
        return rows

    def samples_per_block(self):
        return len(next(iter(self.bursts.values()))[0]) if self.bursts else 0

    # ---- writing
    def header_lines(self, rows):
        blocks = self.blocks()
        missing = self.missing_samples()
        nsamp = self.samples_per_block()
        dur = (getattr(self, "t_end", time.time()) - self.t0)
        rec = [
            ("format", FORMAT),
            ("name", self.name),
            ("file", os.path.basename(self.path)),
            ("started", self.started),
            ("finished", self.finished or ""),
            ("duration_s", round(dur, 1)),
            ("status", self.status),
            ("steps_measured", len(rows)),
            ("blocks_averaged", len(blocks)),
            ("blocks_with_samples", len(blocks) - len(missing)),
            ("blocks_without_samples", len(missing)),
            ("samples_per_block", nsamp or ""),
            ("overloaded_blocks", sum(1 for _, b in blocks if b.get("ovl"))),
        ]
        if rows:
            best = max(rows, key=lambda r: r["amp_fs"])
            rec += [("max_amp_dbfs", round(best["amp_dbfs"], 3)), ("max_at_hz", round(best["f_hz"], 4))]
        out = ["# Photoacoustic detector: recorded frequency sweep", "#", "# [record]"]
        out += ["# %s: %s" % (k, _fmt(v)) for k, v in rec]
        sections = list(self.meta)
        if self.status_msgs:
            sections.append(("device status at start", _flatten(self.status_msgs[0])))
            if len(self.status_msgs) > 1:
                sections.append(("device status at end", _flatten(self.status_msgs[-1])))
        sections.append(("columns of [analysed]", ANALYSED_COLUMNS))
        sections.append(("columns of [blocks]", BLOCK_COLUMNS))
        sections.append(("columns of [samples]", [
            ("step", "frequency step"), ("seq", "block number, as in [blocks]"),
            ("s0000 .. s%04d" % (max(nsamp, 1) - 1),
             "raw microphone samples of the block at the ADC rate (firmware.fs), int16: divide by 32768 for a "
             "fraction of full scale; the top 16 bits of the ADC's 24-bit samples, so a block's lock-in "
             "recomputed from them differs slightly from amp_fs in [blocks]")]))
        for title, items in sections:
            out.append("#")
            out.append("# [%s]" % title)
            pairs = items.items() if isinstance(items, dict) else items
            out += ["# %s: %s" % (k, _fmt(v)) for k, v in pairs]
        out.append("#")
        return out

    def write(self, path=None):
        """Write the record; a temporary file is renamed at the end, so a failed write leaves no partial file.
        Returns the path."""
        path = path or self.path
        rows = self.analyse()
        lines = self.header_lines(rows)
        folder = os.path.dirname(os.path.abspath(path))
        fd, tmp = tempfile.mkstemp(prefix=".record-", suffix=".tmp", dir=folder)
        try:
            with os.fdopen(fd, "w", encoding="utf-8", newline="\n") as fh:
                fh.write("\n".join(lines) + "\n")
                # analysed
                fh.write("[analysed]\n")
                cols = [c for c, _ in ANALYSED_COLUMNS]
                fh.write("\t".join(cols) + "\n")
                for r in rows:
                    fh.write("\t".join((
                        str(r["step"]), "%.4f" % r["f_set_hz"], "%.4f" % r["f_hz"], str(r["p_bclk"]),
                        str(r["n_blocks"]), "%.7g" % r["amp_fs"], "%.3f" % r["amp_dbfs"], "%.3f" % r["amp_rel_db"],
                        "%.6g" % r["amp_mpa_peak"], "%.3f" % r["phase_deg"],
                        "" if math.isnan(r["amp_sem_fs"]) else "%.4g" % r["amp_sem_fs"],
                        "" if math.isnan(r["phase_sem_deg"]) else "%.4g" % r["phase_sem_deg"],
                        "%.7g" % r["re_fs"], "%.7g" % r["im_fs"],
                        "%.1f" % r["adc_gain_db"], str(r["light"]), str(r["overloads"]), str(r["freq_mismatch"]),
                    )) + "\n")
                # blocks
                fh.write("[blocks]\n")
                fh.write("\t".join(c for c, _ in BLOCK_COLUMNS) + "\n")
                for k, b in self.blocks():
                    bu = self.bursts.get(b["seq"])
                    fh.write("\t".join((
                        str(k), str(b["seq"]), str(b.get("t", "")), "%.3f" % b.get("_pc_t", 0.0),
                        "%.4f" % b.get("freq", 0.0), _fmt(bool(b.get("light"))), "%.1f" % b.get("gain", 0.0),
                        _fmt(b.get("i")), _fmt(b.get("q")), "%.7g" % b["amp"], "%.3f" % pa_limits.dbfs(b["amp"]),
                        "%.3f" % b["ph"], _fmt(b.get("rms")), _fmt(b.get("rms_db")), _fmt(b.get("dc")),
                        _fmt(b.get("peak")), _fmt(bool(b.get("ovl"))), _fmt(bool(b.get("clk"))),
                        _fmt(b.get("mod_seq")), ("%.6f" % bu[1]["ph0"]) if bu and "ph0" in bu[1] else "",
                        _fmt(bu[1].get("p_bclk")) if bu else "", "1" if bu else "0",
                    )) + "\n")
                # samples
                fh.write("[samples]\n")
                fh.write("\t".join(["step", "seq"] + ["s%04d" % i for i in range(self.samples_per_block())]) + "\n")
                for k, b in self.blocks():
                    bu = self.bursts.get(b["seq"])
                    if bu is None:
                        continue
                    fh.write("%d\t%d\t" % (k, b["seq"]) + "\t".join(map(str, bu[0].tolist())) + "\n")
            os.replace(tmp, path)
            self.written = path
        except BaseException:
            try:
                os.remove(tmp)
            except OSError:
                pass
            raise
        return path


# ---------------------------------------------------------------- reading
def _num(s):
    if s == "":
        return float("nan")
    try:
        return int(s)
    except ValueError:
        try:
            return float(s)
        except ValueError:
            return s


def read_record(path):
    """Read a record back. Returns a dict:
        meta      {"section.key": "value"} from the header (section titles as written)
        analysed  {column: numpy array}
        blocks    {column: numpy array}
        samples   {seq: numpy int16 array}
        sample_step {seq: step}
    """
    meta, tables, samples, sample_step = {}, {}, {}, {}
    section, head, cols = "", None, None
    with open(path, encoding="utf-8") as fh:
        for line in fh:
            line = line.rstrip("\r\n")
            if head is None and line.startswith("#"):
                t = line[1:].strip()
                if t.startswith("[") and t.endswith("]"):
                    section = t[1:-1]
                elif ": " in t:
                    k, v = t.split(": ", 1)
                    meta["%s.%s" % (section, k)] = v
                continue
            if line.startswith("[") and line.endswith("]") and line[1:-1] in SECTIONS:
                head, cols = line[1:-1], None
                continue
            if not line or head is None:
                continue
            parts = line.split("\t")
            if cols is None:
                cols = parts
                if head != "samples":
                    tables[head] = {c: [] for c in cols}
                continue
            if head == "samples":
                seq = int(parts[1])
                sample_step[seq] = int(parts[0])
                samples[seq] = np.array(parts[2:], dtype=np.int16)
            else:
                for c, v in zip(cols, parts):
                    tables[head][c].append(_num(v))
    out = {"meta": meta, "samples": samples, "sample_step": sample_step}
    for name in ("analysed", "blocks"):
        out[name] = {c: np.array(v) for c, v in tables.get(name, {}).items()}
    return out


def lockin_from_samples(x, ph0, p_bclk, bclk_per_frame=pa_limits.BCLK_PER_FRAME):
    """Recompute a block's lock-in from its raw samples (fraction of full scale) the way the firmware does:
    reference phase ph0 + n * 64 / P cycles, DC removed. Returns (amp_fs, phase_deg = lag)."""
    x = np.asarray(x, dtype=np.float64)
    n = np.arange(len(x))
    th = 2 * np.pi * (ph0 + n * bclk_per_frame / p_bclk)
    c, s = np.cos(th), np.sin(th)
    i = np.dot(x, c) - x.mean() * c.sum()
    q = np.dot(x, s) - x.mean() * s.sum()
    return 2 * math.hypot(i, q) / len(x), math.degrees(math.atan2(q, i))

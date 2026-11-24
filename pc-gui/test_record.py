"""
test_record.py - end-to-end test of the recorded sweep against the simulated detector (no hardware needed).

    pc-gui\\.venv\\Scripts\\python.exe pc-gui\\test_record.py

Runs the real GUI off-screen in a temporary folder with pa_sim's detector, 10x faster than real time. It
connects, switches the modulation on and presses "Record sweep..." (the test answers the dialogs), then checks
the file: the header, the analysed amplitude and phase against the simulated cell and against the GUI's own
sweep plot, every averaged block and its raw samples (the lock-in recomputed from the samples matches the
block), and that the settling blocks are left out. Then: an aborted record, a dark record with an existing
name refused once, a record cut short by closing the window, a record with a block whose samples were lost,
and the file-name rules. Prints PASS/FAIL per check; exit code 0 when everything passes.
"""
import base64
import math
import os
import sys
import tempfile
import time

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
os.environ["QT_QPA_PLATFORM"] = "offscreen"
os.environ.setdefault("PA_SIM_SPEED", "10")
WORK = tempfile.mkdtemp(prefix="pa-record-test-")
os.chdir(WORK)                                  # the data folder is created in the launch folder

import numpy as np                              # noqa: E402
from PySide6.QtWidgets import QApplication, QInputDialog, QMessageBox   # noqa: E402

import pa_gui                                   # noqa: E402
import pa_limits                                # noqa: E402
import pa_record                                # noqa: E402
import pa_sim                                   # noqa: E402

failures = []


def check(ok, what):
    print("%s  %s" % ("PASS" if ok else "FAIL", what))
    if not ok:
        failures.append(what)
    return ok


# ---- the dialogs answer from these queues
names = []          # what the user types into the name dialog (None = Cancel)
asked = []          # questions shown
refuse = []         # substrings of questions to answer No (once each)


def fake_get_text(parent, title, label, mode=None, text=""):
    n = names.pop(0) if names else None
    return ("", False) if n is None else (n, True)


def fake_question(parent, title, text, *a, **k):
    asked.append(text)
    for s in list(refuse):
        if s in text:
            refuse.remove(s)
            return QMessageBox.StandardButton.No
    return QMessageBox.StandardButton.Yes


QInputDialog.getText = staticmethod(fake_get_text)
QMessageBox.question = staticmethod(fake_question)
QMessageBox.warning = staticmethod(lambda *a, **k: asked.append("warning: " + str(a[2] if len(a) > 2 else "")))

pa_sim.enable()
app = QApplication([])


def pump(cond, timeout):
    t_end = time.time() + timeout
    while time.time() < t_end:
        app.processEvents()
        if cond():
            return True
        time.sleep(0.003)
    app.processEvents()
    return cond()


def lag_diff(a, b):
    return abs((a - b + 180.0) % 360.0 - 180.0)


def record_and_wait(win, timeout):
    win.btn_record.click()
    started = pump(lambda: win.record is not None, 2)
    done = pump(lambda: win.record is None and win.sweep is None, timeout)
    return started and done


# ------------------------------------------------------------------ unit checks
check(pa_record.clean_name("  my sweep: test/1 .txt ") == "my sweep_ test_1", "clean_name replaces : / and drops .txt")
check(pa_record.clean_name("CON") == "_CON", "clean_name avoids reserved device names")
check(pa_record.clean_name("  .txt") == "", "clean_name of nothing usable is empty")
check(pa_record.clean_name("a<b>c|d?e*f\"g") == "a_b_c_d_e_f_g", "clean_name replaces every forbidden character")

# ------------------------------------------------------------------ GUI
win = pa_gui.MainWindow()
expected_dir = os.path.join(WORK, pa_record.DATA_DIR_NAME)
check(os.path.normcase(os.path.realpath(win.data_dir)) == os.path.normcase(os.path.realpath(expected_dir))
      and os.path.isdir(expected_dir), "data folder photoacoustic-data created in the launch folder")
check(win.windowTitle().endswith(pa_gui.GUI_VERSION), "window title shows GUI %s" % pa_gui.GUI_VERSION)

win.refresh_ports()
i = win.port_combo.findData(pa_sim.PORT)
check(i >= 0, "simulated port listed")
win.port_combo.setCurrentIndex(i)
win.toggle_connect()
check(pump(lambda: win.fw is not None, 5), "firmware id received from the simulator")
model = win.dev.ser.model
win.gain_spin.setValue(12.0)
win.set_light(True)
check(pump(lambda: win.light_on, 3), "modulation on")

# ---- 1: a complete record
win.sweep_from.setValue(1000)
win.sweep_to.setValue(6000)
win.sweep_points.setValue(12)
win.chk_sweep_log.setChecked(True)
win.sweep_settle.setValue(2)
win.sweep_blocks.setValue(5)
plan = win.sweep_plan()[0]
names.append("  my sweep: test/1 .txt ")
t0 = time.time()
check(record_and_wait(win, 90), "record 1 ran and was written (%.1f s)" % (time.time() - t0))
path = os.path.join(win.data_dir, "my sweep_ test_1.txt")
if check(os.path.isfile(path), "record 1 saved as %s" % os.path.basename(path)):
    r = pa_record.read_record(path)
    m, an, bl, sa = r["meta"], r["analysed"], r["blocks"], r["samples"]
    nst = len(plan)
    check(m.get("record.format") == pa_record.FORMAT, "header: format")
    check(m.get("record.status") == "complete", "header: status complete")
    check(m.get("record.name") == "my sweep_ test_1", "header: name")
    check(m.get("software.gui") == "pa_gui %s" % pa_gui.GUI_VERSION, "header: GUI version")
    check(m.get("device.firmware.version") == pa_sim.FW_VERSION and m.get("device.port") == pa_sim.PORT,
          "header: firmware id and port")
    check(m.get("sweep.frequencies_planned") == str(nst) and m.get("sweep.settle_blocks") == "2"
          and m.get("sweep.average_blocks") == "5", "header: sweep settings")
    check(m.get("measurement.adc_gain_db") == "12" and m.get("measurement.modulation_on_at_start") == "1",
          "header: gain and modulation state")
    check("device status at start.adc.clock_ok" in m and "device status at end.seq" in m,
          "header: device status at start and at end")
    check(m.get("calibration.mic_sensitivity_dbv_pa") == "-38", "header: calibration")
    check("gui settings.fft_len_s" in m and "columns of [analysed].amp_fs" in m, "header: GUI settings, columns")
    check(m.get("record.blocks_averaged") == str(nst * 5) and m.get("record.blocks_without_samples") == "0",
          "header: %d blocks averaged, all with samples" % (nst * 5))
    with open(path, encoding="utf-8") as fh:
        head = fh.read(20000)
    check(WORK not in head and os.path.expanduser("~") not in head, "header holds no folder path")

    check(len(an["step"]) == nst and np.all(an["n_blocks"] == 5), "analysed: %d steps of 5 blocks" % nst)
    check(np.allclose(an["f_hz"], plan), "analysed: the planned frequencies")
    check(np.all(an["light"] == 1) and np.all(an["freq_mismatch"] == 0) and np.all(an["overloads"] == 0),
          "analysed: light on, no frequency mismatch, no overload")
    # the simulated cell has a dip near 1.4 kHz where the signal is close to the noise: judge every step
    # against the noise the simulator adds, 2 * noise_rms / sqrt(N) per block, / sqrt(5) for 5 blocks
    sem_exp = 2 * model.noise_rms_fs / math.sqrt(pa_limits.BLOCK) / math.sqrt(5)
    z_meas = an["re_fs"] + 1j * an["im_fs"]
    z_exp = np.array([e[0] * np.exp(1j * math.radians(e[1])) for e in (model.expected(f, 12.0) for f in an["f_hz"])])
    dev = np.abs(z_meas - z_exp) / (sem_exp + 1e-4 * np.abs(z_exp))
    check(np.all(dev < 4.5), "analysed vs simulated cell: every step within %.1f x the noise" % dev.max())
    top = int(np.argmax(np.abs(z_exp)))
    ea = abs(an["amp_fs"][top] / abs(z_exp[top]) - 1)
    ep = lag_diff(an["phase_deg"][top], math.degrees(np.angle(z_exp[top])))
    check(ea < 0.005 and ep < 0.2, "analysed vs simulated cell at the peak (%.0f Hz): amplitude within %.3f %%, "
          "phase within %.3f deg" % (an["f_hz"][top], 100 * ea, ep))
    gui = {round(f, 4): (a, p) for f, a, p in win.sweep_result}
    same = all(abs(gui[round(f, 4)][0] - a) <= 1e-9 + 1e-7 * a and lag_diff(gui[round(f, 4)][1], p) < 1e-3
               for f, a, p in zip(an["f_hz"], an["amp_fs"], an["phase_deg"]))
    check(same, "analysed = the GUI's sweep plot")
    imax = int(np.argmax(an["amp_fs"]))
    check(abs(an["f_hz"][imax] - model.f0) < 0.15 * model.f0 and abs(float(m["record.max_at_hz"]) - an["f_hz"][imax]) < 1e-3,
          "maximum at %.0f Hz, near the simulated resonance %.0f Hz" % (an["f_hz"][imax], model.f0))
    check(np.all(an["amp_rel_db"] <= 1e-9) and abs(an["amp_rel_db"].max()) < 1e-9, "amp_rel_db is 0 at the maximum")
    check(np.all(an["amp_sem_fs"] > 0.2 * sem_exp) and np.all(an["amp_sem_fs"] < 3 * sem_exp),
          "standard errors match the simulated noise (%.2g .. %.2g, expected about %.2g)"
          % (an["amp_sem_fs"].min(), an["amp_sem_fs"].max(), sem_exp))
    mpa = np.array([win.cal.amp_fs_to_pa(a, 12.0) * 1e3 for a in an["amp_fs"]])
    check(np.allclose(an["amp_mpa_peak"], mpa, rtol=1e-5), "mPa column follows the calibration")

    check(len(bl["seq"]) == nst * 5, "blocks: %d rows (settling blocks left out)" % (nst * 5))
    by_step = {}
    for k, s in zip(bl["step"], bl["seq"]):
        by_step.setdefault(int(k), []).append(int(s))
    contiguous = all(v == list(range(v[0], v[0] + len(v))) for v in by_step.values())
    gaps = [by_step[k + 1][0] - by_step[k][-1] for k in range(1, nst)]
    check(contiguous and min(gaps) >= 3, "blocks: consecutive within a step, >= 2 settling blocks between steps")
    f_of_step = dict(zip(an["step"], an["f_hz"]))
    check(all(abs(f - f_of_step[k]) < 1e-3 for k, f in zip(bl["step"], bl["fw_freq_hz"])),
          "blocks: firmware frequency = step frequency")
    p_of_step = dict(zip(an["step"], an["p_bclk"]))
    check(all(p == p_of_step[k] for k, p in zip(bl["step"], bl["p_bclk"])), "blocks: period = step period")
    zb = [bl["amp_fs"][j] * np.exp(1j * math.radians(bl["phase_deg"][j])) for j in range(len(bl["seq"]))]
    re_ok = all(abs(np.mean([z for z, k in zip(zb, bl["step"]) if k == s]) - (re + 1j * im)) < 1e-6 * abs(re + 1j * im) + 1e-12
                for s, re, im in zip(an["step"], an["re_fs"], an["im_fs"]))
    check(re_ok, "analysed = vector mean of the recorded blocks")

    check(len(sa) == nst * 5 and all(len(x) == 4800 for x in sa.values()), "samples: one row of 4800 per block")
    # the samples are 16-bit (1 LSB = 3e-5 of full scale): the recomputed lock-in differs from the
    # firmware's 24-bit one by a fraction of a microunit of full scale
    worst = 0.0
    for j, s in enumerate(bl["seq"]):
        x = sa[int(s)] / 32768.0
        a, p = pa_record.lockin_from_samples(x, bl["ph0_cycles"][j], bl["p_bclk"][j])
        zs = a * np.exp(1j * math.radians(p))
        zb = bl["amp_fs"][j] * np.exp(1j * math.radians(bl["phase_deg"][j]))
        worst = max(worst, abs(zs - zb) / (1e-6 + 2e-3 * abs(zb)))
    check(worst < 1.0, "samples: lock-in recomputed from the raw samples matches every block "
                       "(worst %.0f %% of the allowed 1e-6 + 0.2 %%)" % (100 * worst))
    size = os.path.getsize(path) / 1e6
    check(0.5 < size < 5, "file size %.2f MB for %d blocks" % (size, nst * 5))

# ---- 2: an aborted record keeps what was measured
win.sweep_points.setValue(30)
names.append("aborted run")
win.btn_record.click()
pump(lambda: win.record is not None and len(win.record.steps) >= 4, 30)
win.toggle_sweep()                              # "Abort sweep"
check(pump(lambda: win.record is None, 5), "aborted record written")
p2 = os.path.join(win.data_dir, "aborted run.txt")
if check(os.path.isfile(p2), "aborted record saved"):
    r2 = pa_record.read_record(p2)
    check(r2["meta"].get("record.status") == "aborted: aborted by user", "aborted record: status says why")
    nb = len(r2["blocks"]["seq"])
    check(len(r2["analysed"]["step"]) >= 3 and len(r2["samples"]) == nb and nb > 0,
          "aborted record: %d steps, %d blocks, all with samples" % (len(r2["analysed"]["step"]), nb))
check(win.btn_record.isEnabled() and win.btn_sweep.text() == "Run sweep", "buttons back to idle")

# ---- 3: dark record; an existing name is refused once, then another name
win.set_light(False)
pump(lambda: not win.light_on, 3)
win.sweep_points.setValue(3)
asked.clear()
refuse.append("already exists")
names += ["my sweep_ test_1", "dark sweep"]
check(record_and_wait(win, 30), "dark record ran")
check(any("modulation is OFF" in q for q in asked) and any("already exists" in q for q in asked),
      "asked about the light being off and about the existing file")
p3 = os.path.join(win.data_dir, "dark sweep.txt")
if check(os.path.isfile(p3), "dark record saved under the second name"):
    r3 = pa_record.read_record(p3)
    check(np.all(r3["analysed"]["light"] == 0) and r3["analysed"]["amp_fs"].max() < 1e-5,
          "dark record: light off in every block, amplitude at the noise level")
check(pa_record.read_record(path)["meta"].get("record.status") == "complete", "the refused name was not overwritten")

# ---- 4: Cancel in the name dialog records nothing
names.append(None)
before = set(os.listdir(win.data_dir))
win.btn_record.click()
app.processEvents()
check(win.sweep is None and win.record is None and set(os.listdir(win.data_dir)) == before, "Cancel records nothing")

# ---- 5: closing the window while recording saves what there is
win.set_light(True)
pump(lambda: win.light_on, 3)
win.sweep_points.setValue(20)
names.append("closed early")
win.btn_record.click()
pump(lambda: win.record is not None and len(win.record.steps) >= 2, 20)
win.close()
p5 = os.path.join(win.data_dir, "closed early.txt")
if check(os.path.isfile(p5), "record saved when the window closed"):
    check(pa_record.read_record(p5)["meta"].get("record.status") == "aborted: window closed",
          "closed record: status says why")

# ---- 6: a block whose raw samples were lost (without the GUI)
sim = pa_sim.SimSerial(speed=1)
sim.close()
sim.light = True
rec = pa_record.SweepRecord("lost", os.path.join(WORK, "lost.txt"), [("note", [("k", "v")])], win.cal)
p, f = pa_limits.snap_frequency(2000)
sim.p, sim.freq = p, f
rec.new_step(2000, f, p)
rec.begin_average()
for j in range(4):
    d, b = sim.make_block()
    rec.add_block(d)
    if j != 2:
        rec.add_burst(np.frombuffer(base64.b64decode(b["b64"]), "<i2"), b)
rec.finish("complete")
rec.write()
r6 = pa_record.read_record(rec.path)
check(r6["meta"]["record.blocks_without_samples"] == "1" and list(r6["blocks"]["samples"]) == [1, 1, 0, 1]
      and len(r6["samples"]) == 3, "lost samples: counted in the header, marked in [blocks], row left out")
check(r6["meta"].get("note.k") == "v", "metadata sections passed through")

print()
print("%d check(s) failed: %s" % (len(failures), "; ".join(failures)) if failures else "all checks passed")
print("test folder:", WORK)
sys.exit(1 if failures else 0)

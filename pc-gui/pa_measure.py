"""
pa_measure.py - headless photoacoustic measurement with the pa-detector firmware.

    py pa_measure.py COM9 [--freqs 500,1000,2000,3332,5000] [--gain 12] [--off 3] [--on 5]
                          [--cycles 1] [--out DIR]

For every frequency: light off for `off` seconds, then `cycles` x (light on for
`on` seconds, light off), recording the lock-in results and the raw sample
stream. Reports, per frequency, the lock-in amplitude and phase with the light
on, the lock-in noise with the light off, the FFT peak at f_mod against the
noise floor next to it, the odd harmonics, and the phase spread between
on/off cycles (the phase origin is the LED rising edge, so it must repeat).
Writes results.csv, spectrum_<f>.csv and, when matplotlib is installed, a
summary PNG into DIR. Always ends with `light off`.
"""
import argparse
import base64
import csv
import json
import math
import os
import sys
import time

import numpy as np
import serial

ap = argparse.ArgumentParser()
ap.add_argument("port")
ap.add_argument("--freqs", default="500,1000,2000,3332,5000")
ap.add_argument("--gain", type=float, default=12.0)
ap.add_argument("--off", type=float, default=3.0)
ap.add_argument("--on", type=float, default=5.0)
ap.add_argument("--cycles", type=int, default=1)
ap.add_argument("--out", default="pa_measure_%s" % time.strftime("%Y%m%d_%H%M%S"))
args = ap.parse_args()
os.makedirs(args.out, exist_ok=True)

FS = 48000
s = serial.Serial(args.port, 115200, timeout=0.1)
rx = b""
t_ping = 0.0


def send(c):
    s.write((c + "\n").encode())


def pump(dur):
    global rx, t_ping
    datas, bursts, other = [], {}, []
    t_end = time.time() + dur
    while time.time() < t_end:
        if time.time() - t_ping > 1.0:
            send("ping")
            t_ping = time.time()
        rx += s.read(65536)
        while b"\n" in rx:
            line, rx = rx.split(b"\n", 1)
            line = line.strip()
            if not line.startswith(b"{"):
                continue
            try:
                m = json.loads(line)
            except ValueError:
                continue
            t = m.get("type")
            if t == "data":
                datas.append(m)
            elif t == "burst":
                bursts[m["seq"]] = (np.frombuffer(base64.b64decode(m["b64"]), "<i2") / 32768.0, m)
            else:
                other.append(m)
    return datas, bursts, other


def wait_for(kind, dur=2.0, **match):
    t_end = time.time() + dur
    while time.time() < t_end:
        _, _, oth = pump(0.2)
        for m in oth:
            if m.get("type") == kind and all(m.get(k) == v for k, v in match.items()):
                return m
    return None


def contiguous(bursts, seqs):
    """Longest run of consecutive bursts -> one sample array (or None)."""
    seqs = sorted(q for q in seqs if q in bursts)
    best, run = [], []
    for q in seqs:
        if run and q != run[-1] + 1:
            best, run = max(best, run, key=len), []
        run.append(q)
    best = max(best, run, key=len)
    if len(best) < 2:
        return None, None
    x = np.concatenate([bursts[q][0] for q in best])
    return x, [bursts[q][1] for q in best]


def spectrum(x):
    n = len(x)
    w = np.hanning(n)
    mag = 2 * np.abs(np.fft.rfft(x * w)) / w.sum()
    f = np.fft.rfftfreq(n, 1 / FS)
    return f, 20 * np.log10(np.maximum(mag, 1e-12))


def peak_and_noise(f, db, f0, span=200.0):
    df = f[1]
    i = int(round(f0 / df))
    if i < 3 or i + 3 >= len(db):
        return -200.0, -200.0
    pk = float(db[i - 2:i + 3].max())
    sel = (np.abs(f - f0) < span) & (np.abs(f - f0) > 4 * df)
    return pk, float(np.median(db[sel]))


def fold(x, metas, p):
    """Synchronous average over the modulation period; phase 0 = LED rising edge.
    At most one bin per sample of the period, so no bin stays empty."""
    bins = max(8, min(64, int(p / 64)))
    acc, cnt = np.zeros(bins), np.zeros(bins)
    off = 0
    for m in metas:
        n = m["n"]
        ph = (m["ph0"] + np.arange(n) * 64.0 / p) % 1.0
        b = (ph * bins).astype(int) % bins
        seg = x[off:off + n]
        acc += np.bincount(b, weights=seg, minlength=bins)
        cnt += np.bincount(b, minlength=bins)
        off += n
    out = acc / np.maximum(cnt, 1)
    out[cnt == 0] = np.nan
    return out


def zmean(datas, light):
    z = np.array([d["amp"] * np.exp(1j * math.radians(d["ph"])) for d in datas if d["light"] == light])
    return z


rows = []
figs = []
try:
    send("light off")
    send("set burst 1")
    send("set heartbeat 5")
    send("set gain %.1f" % args.gain)
    send("start")
    pump(0.5)
    send("id")
    ident = wait_for("id")
    if not ident:
        print("no firmware answer")
        sys.exit(1)
    print("firmware %s %s on %s, gain %.1f dB" % (ident["fw"], ident["version"], args.port, args.gain))

    for f_req in [float(v) for v in args.freqs.split(",")]:
        send("set freq %.4f" % f_req)
        ack = wait_for("ack", key="freq")
        f_mod, p = ack["value"], ack["p_bclk"]
        pump(0.3)
        d_off, b_off, _ = pump(args.off)
        x_off, m_off = contiguous(b_off, [d["seq"] for d in d_off if not d["light"]])
        z_off = zmean(d_off, False)

        z_cycles, x_on, m_on, d_on_all, syncs = [], None, None, [], []
        for c in range(args.cycles):
            send("light on")
            pump(0.6)                                  # arming + the start block + settle
            d_on, b_on, oth = pump(args.on)
            send("status")
            st = wait_for("status")
            syncs.append(st.get("mod_sync") if st else "?")
            send("light off")
            pump(0.4)
            on = [d for d in d_on if d["light"]]
            d_on_all += on
            z = zmean(on, True)
            if len(z):
                z_cycles.append(z.mean())
            if x_on is None:
                x_on, m_on = contiguous(b_on, [d["seq"] for d in on])

        z_on = zmean(d_on_all, True)
        zm = z_on.mean() if len(z_on) else 0
        amp_on = 20 * math.log10(abs(zm) + 1e-12)
        ph_on = math.degrees(np.angle(zm)) if abs(zm) > 0 else float("nan")
        spread_blocks = float(np.std(np.degrees(np.angle(z_on / zm)))) if abs(zm) > 0 else float("nan")
        noise_off = 20 * math.log10(np.mean(np.abs(z_off)) + 1e-12) if len(z_off) else float("nan")
        # standard error of the on-mean: the noise of the averaged phasor
        sem = 20 * math.log10(np.std(z_on) / math.sqrt(len(z_on)) + 1e-12) if len(z_on) > 1 else float("nan")
        cyc_ph = [math.degrees(np.angle(z)) for z in z_cycles]
        cyc_spread = (max(cyc_ph) - min(cyc_ph)) if len(cyc_ph) > 1 else 0.0

        pk_on = nf_on = pk_off = nf_off = float("nan")
        h3 = h5 = float("nan")
        if x_on is not None:
            f, db = spectrum(x_on)
            pk_on, nf_on = peak_and_noise(f, db, f_mod)
            h3, _ = peak_and_noise(f, db, 3 * f_mod) if 3 * f_mod < FS / 2 else (float("nan"), 0)
            h5, _ = peak_and_noise(f, db, 5 * f_mod) if 5 * f_mod < FS / 2 else (float("nan"), 0)
            with open(os.path.join(args.out, "spectrum_%.0fHz.csv" % f_mod), "w", newline="") as fh:
                w = csv.writer(fh)
                w.writerow(["freq_hz", "light_on_dbfs", "light_off_dbfs"])
                if x_off is not None:
                    f2, db2 = spectrum(x_off[:len(x_on)]) if len(x_off) >= len(x_on) else spectrum(x_off)
                    db_off_i = np.interp(f, f2, db2)
                else:
                    db_off_i = np.full_like(db, np.nan)
                for k in range(0, len(f), max(1, len(f) // 20000)):
                    w.writerow(["%.3f" % f[k], "%.2f" % db[k], "%.2f" % db_off_i[k]])
        if x_off is not None:
            f2, db2 = spectrum(x_off)
            pk_off, nf_off = peak_and_noise(f2, db2, f_mod)

        row = dict(f_req=f_req, f_mod=f_mod, p_bclk=p, blocks_on=len(z_on), blocks_off=len(z_off),
                   lockin_on_dbfs=amp_on, lockin_phase_deg=ph_on, block_phase_std_deg=spread_blocks,
                   lockin_off_dbfs=noise_off, lockin_mean_noise_dbfs=sem,
                   fft_peak_on_dbfs=pk_on, fft_noise_on_dbfs=nf_on, fft_snr_db=pk_on - nf_on,
                   fft_peak_off_dbfs=pk_off, h3_dbfs=h3, h5_dbfs=h5,
                   cycles=len(cyc_ph), cycle_phase_spread_deg=cyc_spread, mod_sync=",".join(syncs))
        rows.append(row)
        print("f_mod %8.3f Hz (P %5d): lock-in ON %.1f dBFS @ %+7.2f deg (block std %.2f deg, mean noise %.1f dBFS) | "
              "OFF %.1f dBFS | FFT peak %.1f dBFS, floor %.1f, SNR %.1f dB | 3f %.1f 5f %.1f | sync %s%s" % (
                  f_mod, p, amp_on, ph_on, spread_blocks, sem, noise_off, pk_on, nf_on, pk_on - nf_on, h3, h5,
                  row["mod_sync"], ("  cycle phases %s (spread %.2f deg)" % (["%+.2f" % v for v in cyc_ph], cyc_spread))
                  if len(cyc_ph) > 1 else ""))
        if x_on is not None:
            figs.append((f_mod, p, spectrum(x_on), spectrum(x_off) if x_off is not None else None, fold(x_on, m_on, p)))

    with open(os.path.join(args.out, "results.csv"), "w", newline="") as fh:
        w = csv.DictWriter(fh, fieldnames=list(rows[0].keys()))
        w.writeheader()
        for r in rows:
            w.writerow(r)
    try:
        import matplotlib
        matplotlib.use("Agg")
        import matplotlib.pyplot as plt
        n = len(figs)
        fig, axes = plt.subplots(n, 2, figsize=(14, 3.2 * n), squeeze=False)
        for k, (f_mod, p, (f, db), spoff, fo) in enumerate(figs):
            ax = axes[k][0]
            if spoff is not None:
                ax.semilogx(spoff[0][1:], spoff[1][1:], color="0.6", lw=0.6, label="light off")
            ax.semilogx(f[1:], db[1:], color="C3", lw=0.6, label="light on")
            ax.axvline(f_mod, color="k", ls=":", lw=0.8)
            ax.set_xlim(20, FS / 2)
            ax.set_ylim(-140, 0)
            ax.set_ylabel("dBFS (sine peak)")
            ax.set_title("f_mod %.3f Hz, gain %.1f dB" % (f_mod, args.gain), fontsize=10)
            ax.legend(loc="upper right", fontsize=8)
            ax = axes[k][1]
            ax.plot((np.arange(len(fo)) + 0.5) / len(fo), fo * 1e3, color="C0", marker=".")
            ax.axvspan(0, 0.5, color="gold", alpha=0.25, label="LED on")
            ax.set_xlabel("modulation phase (cycles from the LED rising edge)")
            ax.set_ylabel("synchronous average (mFS)")
            ax.legend(loc="upper right", fontsize=8)
        axes[-1][0].set_xlabel("Hz")
        fig.tight_layout()
        png = os.path.join(args.out, "summary.png")
        fig.savefig(png, dpi=110)
        print("plot:", os.path.abspath(png))
    except ImportError:
        print("matplotlib not installed: no plot")
    print("results:", os.path.abspath(os.path.join(args.out, "results.csv")))
finally:
    send("light off")
    time.sleep(0.3)
    pump(0.3)
    s.close()
    print("light off sent, port closed")

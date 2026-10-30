"""
pa_smoke.py - command-line check of a connected photoacoustic detector.

    py pa_smoke.py COM5 [--blocks 30] [--light] [--gain 12]

Connects, asks for `id`, runs the self-test, streams N blocks and checks that
the sequence numbers are contiguous, that every burst decodes to 4800 int16
samples and that the Pico keeps real time (100 ms per block on its own clock).
With --light it switches the modulation on for 2 s (a sample may be in the
setup: the LED current is whatever the current source is set to) and reports
the lock-in result and the modulator sync state. Always ends with `light off`.
"""
import argparse
import base64
import json
import math
import sys
import time

import numpy as np
import serial

ap = argparse.ArgumentParser()
ap.add_argument("port")
ap.add_argument("--blocks", type=int, default=30)
ap.add_argument("--light", action="store_true")
ap.add_argument("--gain", type=float, default=12.0)
ap.add_argument("--freq", type=float, default=1000.0)
args = ap.parse_args()

s = serial.Serial(args.port, 115200, timeout=0.1)
rx = b""
t_ping = 0.0
fails = 0


def send(c):
    s.write((c + "\n").encode())


def pump(dur):
    """Read for dur seconds; returns (data msgs, {seq: samples}, other msgs)."""
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
                if line:
                    other.append({"type": "raw", "line": line.decode(errors="replace")})
                continue
            try:
                m = json.loads(line)
            except ValueError:
                other.append({"type": "badjson", "line": line[:80].decode(errors="replace")})
                continue
            t = m.get("type")
            if t == "data":
                datas.append(m)
            elif t == "burst":
                try:
                    bursts[m["seq"]] = (np.frombuffer(base64.b64decode(m["b64"]), "<i2"), m)
                except ValueError:
                    other.append({"type": "badburst", "seq": m.get("seq")})
            else:
                other.append(m)
    return datas, bursts, other


def check(cond, text):
    global fails
    print(("  ok   " if cond else "  FAIL ") + text)
    if not cond:
        fails += 1


def wait_for(kind, dur=2.0, **match):
    t_end = time.time() + dur
    found = None
    while time.time() < t_end and found is None:
        _, _, oth = pump(0.2)
        for m in oth:
            if m.get("type") == kind and all(m.get(k) == v for k, v in match.items()):
                found = m
    return found


try:
    send("light off")
    send("set burst 1")
    send("set heartbeat 5")
    send("set gain %.1f" % args.gain)
    send("start")
    pump(0.5)
    send("id")
    ident = wait_for("id")
    check(ident is not None, "id answered: %s" % json.dumps(ident) if ident else "no id answer")
    if ident is None:
        sys.exit(1)
    print("  firmware %s %s proto %s fs %s block %s adc=%s driver=%s sysclk=%s reset=%s" % (
        ident.get("fw"), ident.get("version"), ident.get("proto"), ident.get("fs"), ident.get("block"),
        ident.get("adc"), ident.get("driver"), ident.get("sysclk_hz"), ident.get("reset_reason")))
    check(ident.get("driver") == "vccs" and ident.get("adc") is True, "vccs driver and ADC detected")

    send("set freq %.4f" % args.freq)
    ack = wait_for("ack", key="freq")
    check(ack is not None, "freq ack: %s" % ack)

    send("selftest")
    st = wait_for("selftest", dur=3.0)
    check(st is not None, "selftest answered")
    if st:
        for r in st["results"]:
            print("    %-20s %-5s %s" % (r["name"], r["status"], r["detail"]))
        check(all(r["status"] != "fail" for r in st["results"]), "no self-test failures")

    print("streaming %d blocks ..." % args.blocks)
    datas, bursts, other = pump(args.blocks / 10.0 + 0.5)
    seqs = [d["seq"] for d in datas]
    gaps = sum(1 for a, b in zip(seqs, seqs[1:]) if b != a + 1)
    check(len(datas) >= args.blocks, "%d data messages (expected >= %d)" % (len(datas), args.blocks))
    check(gaps == 0, "%d sequence gaps" % gaps)
    check(all(q in bursts for q in seqs), "%d of %d data blocks have a burst" % (sum(q in bursts for q in seqs), len(seqs)))
    check(all(len(b[0]) == ident["block"] for b in bursts.values()), "every burst has %s samples" % ident["block"])
    if len(datas) >= 2:
        ms = (datas[-1]["t"] - datas[0]["t"]) / (datas[-1]["seq"] - datas[0]["seq"])
        check(abs(ms - 100.0) < 0.5, "%.3f ms per block on the Pico clock" % ms)
    bad = [m for m in other if m.get("type") in ("badjson", "badburst", "raw")]
    check(not bad, "%d malformed lines: %s" % (len(bad), bad[:3]))
    logs = [m for m in other if m.get("type") in ("log", "error")]
    for m in logs:
        print("    firmware:", m)
    if datas:
        d = datas[-1]
        print("  last block: rms %.1f dBFS, peak %d, dc %.5f, amp %.1f dBFS @ %.1f deg, light=%s" % (
            d["rms_db"], d["peak"], d.get("dc", 0), d["amp_db"], d["ph"], d["light"]))
    x = np.concatenate([bursts[q][0] for q in sorted(bursts)]) / 32768.0 if bursts else np.zeros(1)
    print("  burst stream: %.1f dBFS rms, %.4f mean, %.1f %% clipped" % (
        20 * math.log10(x.std() + 1e-12), x.mean(), 100 * np.mean(np.abs(x) > 0.999)))

    send("status")
    stat = wait_for("status")
    if stat:
        print("  status: overruns=%s skipped=%s i2c_errors=%s tx_drop=%s/%s mod_sync=%s block_period_us=%s" % (
            stat.get("overruns"), stat.get("blocks_skipped"), stat.get("i2c_errors"), stat.get("tx_drop_burst"),
            stat.get("tx_drop_data"), stat.get("mod_sync"), stat.get("block_period_us")))
        check(stat.get("overruns") == 0 and stat.get("tx_drop_burst") == 0, "no overruns, no dropped bursts")

    if args.light:
        print("light on for 2 s ...")
        send("light on")
        ack = wait_for("ack", key="light")
        check(ack is not None and ack.get("value") is True, "light ack: %s" % ack)
        d_on, b_on, oth = pump(2.0)
        send("selftest")
        st2 = wait_for("selftest", dur=3.0)
        send("light off")
        wait_for("ack", key="light")
        on = [d for d in d_on if d["light"]]
        check(len(on) >= 15, "%d blocks with light on" % len(on))
        if on:
            z = np.array([d["amp"] * np.exp(1j * math.radians(d["ph"])) for d in on[2:]])
            zm = z.mean()
            print("  lock-in with light: |<z>| %.1f dBFS @ %+.1f deg, block spread %.1f deg, mod_seq %s" % (
                20 * math.log10(abs(zm) + 1e-12), math.degrees(np.angle(zm)),
                float(np.std(np.degrees(np.angle(z / zm)))) if abs(zm) > 0 else 0, on[-1].get("mod_seq")))
        if st2:
            for r in st2["results"]:
                if r["name"] in ("mod_pin", "mod_sync", "modulator_state"):
                    print("    %-20s %-5s %s" % (r["name"], r["status"], r["detail"]))
            check(all(r["status"] != "fail" for r in st2["results"]), "no self-test failures with light on")
finally:
    send("light off")
    time.sleep(0.3)
    pump(0.3)
    s.close()
    print("light off sent, port closed. %s" % ("ALL OK" if fails == 0 else "%d FAILURES" % fails))

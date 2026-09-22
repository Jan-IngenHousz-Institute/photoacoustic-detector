"""
main.py - photoacoustic-detector shield on a Raspberry Pi Pico (RP2040).

1. Checks that the TLV320ADC6120 answers on I2C (GP4 = SDA, GP5 = SCL).
2. Brings the ADC up: I2S slave, channel 1 differential mic input, 48 kHz.
3. Starts the Pico I2S master on GP6 (BCLK), GP7 (FSYNC), GP8 (SDOUT).
4. Prints the ADC clock/mode status and a running signal level of channel 1.

Copy main.py and TLV320.py to the Pico's filesystem (e.g. with mpremote:
    mpremote cp TLV320.py main.py :
) and open the REPL to watch the output.

The onboard LED blinks fast if the ADC is missing, slow while running.
"""

from machine import I2C, I2S, Pin
from array import array
import math
import time

import TLV320

# ---- pins (from the KiCad PCB) --------------------------------------------
PIN_SDA = 4
PIN_SCL = 5
PIN_BCLK = 6      # I2S sck  (ws must be sck + 1 on the RP2040 port)
PIN_FSYNC = 7     # I2S ws
PIN_SDOUT = 8     # I2S sd (input)
PIN_LED = 25      # Pico H onboard LED

# ---- audio settings -------------------------------------------------------
SAMPLE_RATE = 48_000
BITS = 32                 # 32-bit stereo -> BCLK/FSYNC ratio 64, supported by the ADC
GAIN_DB = 12              # analog gain, 0..42 dB
FRAMES = 2048             # frames per read (stereo -> 2 samples per frame)

led = Pin(PIN_LED, Pin.OUT)


def blink_forever(period_ms):
    while True:
        led.toggle()
        time.sleep_ms(period_ms)


def level_dbfs(buf, step=2, offset=0):
    """RMS and peak of every `step`-th sample, in dBFS (32-bit full scale)."""
    fs = 2147483647.0
    acc = 0.0
    peak = 0
    n = 0
    for i in range(offset, len(buf), step):
        v = buf[i]
        acc += float(v) * v
        a = -v if v < 0 else v
        if a > peak:
            peak = a
        n += 1
    rms = math.sqrt(acc / n) if n else 0.0
    to_db = lambda x: 20 * math.log10(x / fs) if x > 0 else -200.0
    return to_db(rms), to_db(peak)


# ---- 1. I2C and presence check --------------------------------------------
i2c = I2C(0, sda=Pin(PIN_SDA), scl=Pin(PIN_SCL), freq=400_000)
found = i2c.scan()
print("I2C devices:", [hex(a) for a in found])

adc = TLV320.TLV320ADC6120(i2c)
if not adc.detect():
    print("TLV320ADC6120 NOT found at 0x%02X" % TLV320.I2C_ADDR)
    blink_forever(100)
print("TLV320ADC6120 found at 0x%02X" % TLV320.I2C_ADDR)

# ---- 2. ADC bring-up ------------------------------------------------------
adc.reset()
mismatch = adc.verify()
if mismatch:
    print("warning: unexpected reset values (reg, got, expected):", mismatch)
else:
    print("register defaults match TLV320ADC6120 datasheet")

adc.begin(bits=BITS, gain_db=GAIN_DB)      # reset + wake + configure + power up

# ---- 3. Pico I2S master (RX) -----------------------------------------------
i2s = I2S(
    0,
    sck=Pin(PIN_BCLK),
    ws=Pin(PIN_FSYNC),
    sd=Pin(PIN_SDOUT),
    mode=I2S.RX,
    bits=BITS,
    format=I2S.STEREO,
    rate=SAMPLE_RATE,
    ibuf=FRAMES * 2 * (BITS // 8) * 4,
)

time.sleep_ms(50)                          # let the ADC PLL lock to BCLK/FSYNC
st = adc.status()
print("ADC status:", st)
if not st["clock_ok"]:
    print("warning: ADC did not detect a valid FSYNC/BCLK ratio")

# ---- 4. stream and print signal level -------------------------------------
buf = array("i", (0 for _ in range(FRAMES * 2)))   # int32, L/R interleaved
mv = memoryview(buf)

print("streaming channel 1 (left slot)... Ctrl-C to stop")
try:
    t_last = time.ticks_ms()
    while True:
        n = i2s.readinto(mv)
        if n == 0:
            continue
        rms, peak = level_dbfs(buf, step=2, offset=0)
        if time.ticks_diff(time.ticks_ms(), t_last) >= 500:
            t_last = time.ticks_ms()
            led.toggle()
            print("CH1  rms %6.1f dBFS   peak %6.1f dBFS" % (rms, peak))
except KeyboardInterrupt:
    pass
finally:
    i2s.deinit()
    adc.power_down()
    adc.sleep()
    led.off()
    print("stopped, ADC in sleep mode")

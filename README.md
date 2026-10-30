# photoacoustic-detector

A photoacoustic detector built from a TLV320ADC6120 audio-ADC shield on a
Raspberry Pi Pico (RP2040), an IM73A135 MEMS microphone and an on/off
current source driving a 1900 nm LED (Ushio SMBB1900D-1100-02). The Pico
switches the LED with a square wave between 200 Hz and 10 kHz, computes the
lock-in amplitude and phase of the microphone signal at that frequency and
streams the raw samples; the PC shows the FFT, the lock-in result and
frequency sweeps.

```
firmware/      Pico SDK (C) firmware, prebuilt image in firmware/release/
pc-gui/        desktop GUI (PySide6) and command-line tools
measurements/  recorded data (first recording 2026-10-30)
kicad/         shield schematic and PCB
doc/           datasheets (ADC, microphone, TI analog-input app note)
legacy/        superseded MicroPython firmware, reference only
```

## Quick start

### 1. What you need

- The assembled detector: Pico with the shield and the microphone on its
  flex, the LED current source wired to the Pico (control input on GP0,
  pin 1; ground on pin 3), a USB cable.
- A PC with Python 3.10 or newer. On Windows use the python.org interpreter
  (`py -3.13`); see the note on Anaconda below.

Safety: the LED current is set in the current-source hardware. Keep it at or
below 1 A, the LED's continuous rating; the Pico only switches it with a
50 % square wave. The firmware holds GP0 low from boot, switches the light
off when no command has arrived from the PC for 5 s, and when the serial
port is closed. Fit a ~100 kOhm pull-down on the control input so the LED
also stays off while the Pico is in reset or in the bootloader.

### 2. Plug the device in

It enumerates as **pa-detector**, a USB serial port (Windows `COMx`, Linux
`/dev/ttyACMx`, macOS `/dev/cu.usbmodem*`). No driver is needed. If a drive
called `RPI-RP2` appears instead, the Pico is in its bootloader: copy
`firmware/release/pa_detector-1.0.0.uf2` onto that drive and it restarts as
the detector (see "Updating the firmware").

### 3. Install and start the GUI

Pick one:

- **Windows, nothing to install (recommended):** download
  `PhotoacousticDetector-windows.zip` from the
  [releases page](https://github.com/Jan-IngenHousz-Institute/photoacoustic-detector/releases),
  unzip it anywhere and double-click `PhotoacousticDetector.exe`. The zip
  also contains the firmware image and this README.
- **Windows, from this repository:** double-click `pc-gui\run_gui.bat`. On
  first use it installs Python 3.13 if the PC has none (python.org build,
  per user, through winget) and creates a private environment in
  `pc-gui\.venv`; later starts take a second. Do not start the GUI from an
  Anaconda prompt: Anaconda's Python cannot load PySide6 (see
  Troubleshooting); the script skips Anaconda on purpose.
- **Linux / macOS:** `sh pc-gui/run_gui.sh` (needs `python3` 3.10 or newer
  with the venv module; on Linux add yourself to the `dialout` group to open
  the serial port).

Manual alternative with any python.org Python 3.10+:
`pip install -r pc-gui/requirements.txt`, then `python pc-gui/pa_gui.py`.
`python pc-gui/pa_gui.py --check` builds the window off-screen and reports
the versions, useful to verify an installation.

### 4. First measurement

1. Choose the port marked `[pa-detector]` and press **Connect**. The console
   reports `firmware pa-detector 1.0.0` and a status line; the indicators
   for Pico, firmware, ADC, clock and modulator turn green.
2. Press **Self-test...** and **Run device self-test**. Every entry should
   pass; `mic_noise_floor` reads about -80 to -85 dBFS at 12 dB gain in a
   quiet room. **Run light on/off check (2 s)** flashes the LED and reports
   the signal difference at the modulation frequency.
3. In **Excitation (LED current source on GP0)** set the modulation
   frequency (200 Hz to 10 kHz; it snaps to 3.072 MHz / P and the actual
   value is shown next to it). In **ADC (TLV320ADC6120)** set the gain
   (0 to 42 dB); **Auto-range gain** picks it so that the peak sits at
   -12 dBFS.
4. Press **Modulation ON**. The **FFT** tab shows the spectrum of the
   gap-free sample stream with a marker at f_mod and its harmonics, the
   noise floor next to the peak and the SNR; choose 0.1 to 5 s frames and
   the number of averages, **Zoom around f_mod**, **Log frequency**,
   **Export CSV**. The **Lock-in** tab shows amplitude and phase of every
   100 ms block and their running average; **Take dark** with the light
   off stores a dark reference that **Subtract** removes. The **Time
   domain** tab shows the raw block and the synchronous average over one
   modulation period (phase 0 = LED rising edge).
5. The red **LIGHT OFF** button stops the modulation at any time, as does
   closing the GUI or unplugging the USB cable.
6. **Sweep** tab, **Frequency sweep (find cell resonance)**: from/to
   frequency, number of points, log spacing, settle and average blocks,
   **Run sweep**, **Export sweep CSV**.
7. **Logging**: **Start logging** writes one CSV row per block with the
   full settings context. **Save config** / **Load config** keep the GUI
   settings.
8. Units: dBFS (sine peak relative to the ADC full scale) or mPa, using the
   **Calibration** fields (microphone -38 dBV/Pa, ADC 2 Vrms full scale at
   0 dB gain).

### 5. Command line instead of the GUI

```
py pc-gui/pa_smoke.py COM9            # self-test, 30 blocks, timing and protocol checks
py pc-gui/pa_smoke.py COM9 --light    # the same plus 2 s of modulation at 1 kHz
py pc-gui/pa_measure.py COM9 --freqs 500,1000,2000,3332 --off 3 --on 5 --cycles 3 --gain 12
```

`pa_measure.py` records, per frequency, light-off and light-on blocks and
writes `results.csv` (lock-in amplitude and phase with the light on, lock-in
level with the light off, FFT peak against the noise floor in 0.2 Hz bins,
odd harmonics, phase spread between light cycles), the spectra, and a
`summary.png` when matplotlib is installed.

## Reading the numbers

- **dBFS**: 0 dBFS is a full-scale sine, 2 Vrms differential at the ADC
  input at 0 dB gain. With the microphone's -38 dBV/Pa, 94 dB SPL reads
  about -44 dBFS at 0 dB gain. The noise floor is about -84 dBFS RMS at
  12 dB gain; the microphone's own noise (21 dB(A) SPL) sets the floor from
  about 20 dB gain upward.
- **Phase**: the lag of the microphone signal behind the LED rising edge,
  as seen at the ADC output. It repeats between light cycles and sessions
  to about a degree. For the physical response subtract the ADC group
  delay, 360 * f_mod * 17.1 / 48000 degrees, and the high-pass filter's
  lead, atan(12 Hz / f_mod).
- **First recording** (`measurements/2026-10-30_led-on-mic/`, the LED
  pointed at the microphone, 12 dB gain): -97 dBFS at 500 Hz falling to
  -112 dBFS at 3.3 kHz, 28 to 35 dB above the noise in 0.2 Hz bins, phase
  repeatable within 0.6 to 2.7 degrees over three light cycles.

## Updating the firmware

The tested image is `firmware/release/pa_detector-1.0.0.uf2` (also inside
the standalone zip).

1. Get the Pico into its bootloader: with the GUI connected press
   **Update firmware...** in the Tools group (it sends the `bootsel` command
   and disconnects), or hold the BOOTSEL button on the Pico while plugging
   the USB cable in.
2. Copy the UF2 onto the `RPI-RP2` drive that appears (the GUI's status bar
   points it out). The Pico restarts as pa-detector within a few seconds;
   the port number may change, press **Refresh** and connect again.

Building from source, the serial protocol, the clock scheme and the safety
design are documented in `firmware/README.md`.

## Troubleshooting

| Symptom | What to do |
|---|---|
| No serial port, no `RPI-RP2` drive | Try another cable or USB port (data cable, not charge-only). |
| `RPI-RP2` drive, no port | The Pico has no firmware: copy the UF2 from `firmware/release/` onto the drive. |
| Port is shown as `[Pico, MicroPython: old firmware]` | The board still runs the legacy firmware: `mpremote exec "import machine; machine.bootloader()"`, then copy the UF2. |
| "no firmware answer" after Connect | Another program holds the port (close it), or the board is not running the detector firmware. |
| `DLL load failed while importing QtCore` | The GUI was started with Anaconda's Python, whose MSVC runtime is too old for PySide6. Use `run_gui.bat` or the standalone build. Inside conda it works from a dedicated environment: `conda create -n pa -c conda-forge python=3.12 pyside6 pyqtgraph pyserial numpy`, then `conda activate pa`. |
| `adc_*` self-test entries fail | ADC or I2C problem on the shield; `dump` prints the registers. |
| `mic_noise_floor` fails as silent | Microphone unpowered or flex not seated; check MICBIAS (2.75 V) at the connector. |
| Full-scale noise on the microphone channel | The ADC's AREG regulator is oscillating: C5 must be 10 uF (see Hardware notes). |
| Light never comes on | Check the GP0 wiring and the current source; the firmware switches the light off 5 s after the last command, so keep the GUI (or a `ping` every second) running. |

## Measurement principle

The Pico generates the I2S clocks itself from a 76.8 MHz system clock with
integer dividers, so the ADC samples at exactly 48000.000 Hz. A PIO state
machine counts I2S bit clocks (64 per sample), so the modulation period is a
whole number P of bit clocks and f_mod = 3.072 MHz / P exactly (P = 308 to
15360). The modulator is started on a known sample frame and the lock-in
reference on the Pico runs at the same ratio with phase 0 on the LED rising
edge, so the reference never drifts against the light, successive 100 ms
blocks average coherently, and the phase is reproducible. Each block gives
the amplitude and phase of the photoacoustic response, which the PC
averages, plots and logs; the raw samples of every block go to the PC for
the FFT. The lock-in uses the ADC's 24-bit samples; the streamed samples are
16-bit.

## Hardware notes

| Function                   | Pico pin        | Notes                                                        |
|----------------------------|-----------------|--------------------------------------------------------------|
| ADC I2C SDA / SCL          | GP4 / GP5       | on the shield, 2k2 pull-ups to 3V3                           |
| ADC BCLK / FSYNC / SDOUT   | GP6 / GP7 / GP8 | Pico is I2S master: 3.072 MHz, 48 kHz, 24-bit in 32-bit slots |
| ADC GPIO1 ("EXT")          | GP9             | unused (ADC interrupt output)                                 |
| LED current source control | GP0 (pin 1)     | 3.3 V = LED on, 0 V = off; fit a ~100 kOhm pull-down          |
| GND                        | GND (pin 3)     | common ground with the current source                        |

The ADC's internal AREG regulator needs 10 uF + 100 nF (datasheet). The
first PCB has C5 = 1 uF, which made AREG oscillate (2.1 V average, full-scale
noise on the microphone channel); fit a 10 uF 6.3 V X5R there. The KiCad
project in `kicad/` still shows 1 uF.

## Making a release (maintainers)

`pc-gui\build_exe.bat` builds the standalone Windows GUI with PyInstaller
into `pc-gui\dist\PhotoacousticDetector\` and zips it as
`pc-gui\dist\PhotoacousticDetector-windows.zip` (firmware image and README
included). Publish it together with the firmware image:

```
gh release create v1.0.0 pc-gui/dist/PhotoacousticDetector-windows.zip firmware/release/pa_detector-1.0.0.uf2
```

## Legacy

`legacy/firmware-micropython/` is the first, MicroPython firmware. It is no
longer used by the GUI or the tools; see `legacy/README.md`.

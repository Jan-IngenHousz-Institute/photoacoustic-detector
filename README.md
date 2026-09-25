# photoacoustic-detector

Photoacoustic detector built from a TLV320ADC6120 audio ADC shield on a
Raspberry Pi Pico (RP2040), an IM73A135 MEMS microphone, and an
actoPulserX4-48V programmable current source driving a 1900 nm LED
(Ushio SMBB1900D-1100-02).

```
kicad/        shield schematic and PCB (Pico + TLV320ADC6120 + mic connector)
doc/          datasheets (ADC, microphone, TI analog-input app note)
firmware/   MicroPython firmware for the Pico
pc-gui/           PySide6 desktop GUI
```

## Wiring

| Function                | Pico pin | Notes                                          |
|-------------------------|----------|------------------------------------------------|
| ADC I2C SDA / SCL       | GP4 / GP5 | on the shield, 2k2 pull-ups to 3V3            |
| ADC BCLK / FSYNC / SDOUT| GP6 / GP7 / GP8 | Pico is I2S master, 48 kHz 16-bit stereo |
| ADC GPIO1 ("EXT")       | GP9      | unused (ADC interrupt output)                  |
| actopulser SDA / SCL    | GP2 / GP3 | J1 pins 7 / 6, Pico internal pull-ups enabled |
| actopulser LDAC         | GP14     | J1 pin 8, held low                             |
| actopulser PULSEa..d    | GP10..GP13 | J1 pins 1..4                                 |
| GND                     | GND      | J1 pin 5                                       |

Power the actopulser from its own USB-PD supply, not from the Pico. For the
1.4 V LED select 5 V on the actopulser DIP switch: higher rails only heat the
driver MOSFET.

## Firmware (firmware/)

| File           | Purpose                                                        |
|----------------|----------------------------------------------------------------|
| `TLV320.py`    | TLV320ADC6120 driver (presence check, bring-up, gain, status)  |
| `MCP4728.py`   | actopulser DAC driver (0.5 mA per LSB into the 2 ohm sense)    |
| `pulser.py`    | PIO pulse generator locked to the I2S frame clock              |
| `lockin.py`    | 100 ms block lock-in (viper), raw block de-interleave          |
| `main.py`      | application: self-test, serial protocol, safety limits         |
| `level_meter.py` | stand-alone level meter for first bring-up (`import level_meter`) |

Install: flash a MicroPython UF2 (hold BOOTSEL while plugging in, copy the
UF2 onto the RPI-RP2 drive), then

```
pip install mpremote
mpremote cp firmware/TLV320.py firmware/MCP4728.py firmware/pulser.py firmware/lockin.py firmware/main.py :
mpremote reset
```

`main.py` starts automatically at boot and speaks a line protocol over USB
serial (see the docstring in `firmware/main.py`). Safety rules enforced on
the Pico: LED current limited to 2 A only for pulses of at most 10 us at
1 % duty and to 1 A otherwise, light switched off when the PC stops sending
commands for 5 s, and on any firmware exception.

## PC GUI (pc-gui/)

```
pc-gui\run_gui.bat
```

or, with any Python 3.10+ that imports PySide6 cleanly:

```
pip install -r pc-gui/requirements.txt
python pc-gui/pa_gui.py
```

Note for anaconda users: the anaconda base interpreter fails with
`DLL load failed while importing QtCore` because its root folder ships MSVC
runtime 14.44 DLLs that shadow the newer runtime PySide6 6.11 needs. Use the
python.org interpreter (`py -3.13`, which `run_gui.bat` does), or install
PySide6 from conda-forge inside a conda environment instead of pip.

What it does:

- finds Picos on USB, flags one left in BOOTSEL mode, checks the firmware
  answers, and runs a layered self-test (ADC ack, register test, reset
  defaults, clock lock, active mode, FSYNC present, mic noise floor,
  DAC ack and readback, optional light on/off signal check);
- excitation control with live duty, LED and driver power, and refusal of
  settings outside the LED rating;
- lock-in amplitude and phase strip charts with block averaging, dark
  reference subtraction, dBFS or pascal units, SNR from the spectrum;
- spectrum with markers at the modulation frequency and harmonics, raw
  time trace and pulse-synchronous average;
- frequency sweep to locate the acoustic resonance of the cell;
- CSV logging with the full settings context, notes, config save/load.

## Measurement principle

The Pico drives the LED with short current pulses at a modulation frequency
that divides 48 kHz. A PIO state machine derives the pulse timing from the
I2S word clock, so every pulse falls on the same sample index and the
lock-in phase is stable. Each 100 ms block of ADC samples holds a whole
number of modulation periods; its single-bin DFT gives amplitude and phase
of the photoacoustic response, which the PC averages, plots and logs.

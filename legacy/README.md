# legacy/

Superseded material, kept for reference only.

## firmware-micropython/

The first firmware, MicroPython 1.24 on the Pico (version 0.3.0). It spoke
the same line protocol as the current C firmware (`firmware/`), with 16-bit
samples, a fractional-divider I2S clock and an arbitrary modulation phase
origin. It was replaced on 2026-10-30 by the Pico SDK firmware, which the GUI
and the command-line tools now expect (USB PID 0x000A; the GUI labels a
MicroPython Pico as "old firmware").

| File            | Purpose                                                     |
|-----------------|-------------------------------------------------------------|
| `main.py`       | application: protocol, self-test, heartbeat safety          |
| `TLV320.py`     | TLV320ADC6120 driver                                        |
| `modulator.py`  | PIO square wave on GP0 counting I2S bit clocks              |
| `lockin.py`     | block lock-in (viper)                                       |
| `level_meter.py`| stand-alone level meter used during the first bring-up      |
| `MCP4728.py`, `pulser.py` | drivers for the earlier actoPulserX4-48V LED driver (no longer fitted) |

To run it anyway: flash a MicroPython UF2, then
`mpremote cp TLV320.py modulator.py lockin.py main.py :` and `mpremote reset`.
To get a MicroPython Pico back to the bootloader for the current firmware:
`mpremote exec "import machine; machine.bootloader()"`.

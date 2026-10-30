# firmware/ - Pico SDK firmware for the photoacoustic detector

Developer reference. For using the device see the top-level README.

| File                                | Purpose                                                      |
|-------------------------------------|--------------------------------------------------------------|
| `src/main.c`                        | boot order, main loop, block processing                      |
| `src/i2s_rx.pio`, `src/i2s_rx.c`    | PIO I2S master receiver + DMA ping-pong, 100 ms blocks       |
| `src/modulator.pio`, `src/modulator.c` | square wave on GP0 counting I2S bit clocks, started on a known sample frame |
| `src/lockin.c`                      | exact-arithmetic block lock-in on the 24-bit samples         |
| `src/tlv320adc6120.c`               | ADC driver (I2C, timeouts, error counting)                   |
| `src/usb_io.c`, `src/usb_descriptors.c` | TinyUSB CDC with a 32 kB non-blocking TX ring            |
| `src/protocol.c`                    | commands and JSON messages                                   |
| `src/selftest.c`, `src/framecount.pio` | device self-test                                          |
| `src/safety.c`                      | watchdog, heartbeat timer, panic / hard-fault LED-off        |
| `src/board.h`                       | pin map, clock scheme, fixed parameters                      |
| `release/pa_detector-<version>.uf2` | prebuilt image of the tested version                        |

## Build

Prerequisites: Pico SDK 2.2.0 (with its TinyUSB submodule), Arm GNU toolchain
14.2.Rel1, CMake >= 3.17, Ninja, and the prebuilt `pioasm` and `picotool`
from the `pico-sdk-tools` release. The easiest layout is the one the
"Raspberry Pi Pico" VS Code extension uses (`~/.pico-sdk/sdk/2.2.0`,
`toolchain/14_2_Rel1`, `cmake/v3.31.5`, `ninja/v1.12.1`, `tools/2.2.0/pioasm`,
`picotool/2.2.0-a4/picotool`, plus `cmake/pico-vscode.cmake`): the header
block in `CMakeLists.txt` then finds everything.

```
cmake -S firmware -B firmware/build -G Ninja
ninja -C firmware/build
```

(`cmake` and `ninja` must be on PATH, for example
`~/.pico-sdk/cmake/v3.31.5/bin` and `~/.pico-sdk/ninja/v1.12.1`.) Without
the extension layout, pass `-DPICO_SDK_PATH=...`, `-DPICO_TOOLCHAIN_PATH=...`,
`-Dpioasm_DIR=.../pioasm` and `-Dpicotool_DIR=.../picotool`. The result is
`firmware/build/pa_detector.uf2` (about 80 kB of flash, 184 kB of RAM).

Two SDK details worth knowing: pioasm 2.x reserves `next` and `prev` (not
usable as labels), and the RP2040 ROM `cos()`/`sin()` are only valid for
arguments below about 1024 rad.

## Flash

- Running detector firmware: send `bootsel` on its serial port (or open the
  port at 1200 baud). The Pico reboots into the UF2 bootloader and shows up
  as the `RPI-RP2` drive; copy the UF2 there.
- Any Pico: hold BOOTSEL while plugging it in, then copy the UF2 to `RPI-RP2`.
- A Pico still running MicroPython:
  `mpremote exec "import machine; machine.bootloader()"`.

The board enumerates as "pa-detector" (USB serial, VID 0x2E8A, PID 0x000A).

## Clock scheme and phase convention

The system clock is 76.8 MHz (set by the SDK at boot from `SYS_CLK_HZ` and
`PLL_SYS_*` in `CMakeLists.txt`), so the PIO makes the I2S bit clock with an
integer divider: 25 cycles per bit, BCLK = 3.072 MHz, 64 bit clocks per
frame, fs = 48000.000 Hz. The ADC locks its PLL to this jitter-free clock
and delivers 24-bit samples in 32-bit slots.

The modulation period is a whole number P of bit clocks (P = 308 .. 15360,
9974 Hz .. 200 Hz, 0.33 Hz steps at 1 kHz), f_mod = 3.072 MHz / P exactly.
When the light is switched on (or the frequency changes while it is on) the
modulator is restarted at the next 100 ms block boundary and begins on a
known frame of that block; the firmware verifies which one (`mod_sync`,
normally `frame0`; arming runs from RAM in about 1 us) and sets the lock-in
reference so that **phase 0 is the LED rising edge**, with sample n taken at
bit clock 64 n of its frame. The reported phase therefore repeats between
light cycles and sessions. It still includes the ADC's group delay
(`adc_latency_samples` in `id`, 17.1 samples at 48 kHz, i.e.
360 * f_mod * 17.1 / 48000 degrees) and the high-pass filter's lead
(atan(hpf_hz / f_mod)), which the PC can correct.

The lock-in runs on the 24-bit samples; bursts carry the top 16 bits so the
GUI's FFT path is unchanged from the first firmware.

## Serial protocol

Commands are text lines, replies single JSON lines.

| Command                        | Reply / effect                                             |
|--------------------------------|------------------------------------------------------------|
| `id`                           | `{"type":"id", fw, version, proto, fs, block, adc, driver:"vccs", mod_pin, bclk_per_frame, fmin, fmax, sysclk_hz, hpf_hz, adc_latency_samples, mod_edge_frames, uid, reset_reason}` |
| `status`                       | `{"type":"status", adc:{mode, fs, bclk_ratio, clock_ok, ...}, freq, p_bclk, gain_db, light, streaming, burst, heartbeat_s, seq, overruns, blocks_skipped, i2c_errors, tx_drop_burst, tx_drop_data, mod_sync, mod_seq, block_period_us, uptime_s, mod_w_entry, mod_w_armed, mod_arm_us, ...}` |
| `selftest`                     | `{"type":"selftest","results":[{name, status, detail}, ...]}`   |
| `set freq <Hz>`                | snapped to 3.072 MHz / P; ack carries `value` (actual Hz) and `p_bclk` |
| `set gain <dB>`                | ADC channel gain 0 .. 42 dB in 0.5 dB steps                |
| `set burst <N>`                | raw samples every N blocks (0 = off, default 1 = gap-free) |
| `set heartbeat <s>`            | light off after s seconds without a command (0 = off)      |
| `set hpf 12\|96\|384\|0`       | ADC high-pass corner (0 = all-pass)                        |
| `set ch2 0\|1`                 | enable ADC channel 2 (open inputs) for the noise comparison |
| `light on\|off`                | modulation on GP0                                          |
| `start` / `stop`               | data streaming                                             |
| `ping`                         | keeps the heartbeat alive                                  |
| `dump`                         | ADC page-0 registers                                       |
| `bootsel` / `reset`            | reboot into the UF2 bootloader / reboot                    |

Every 100 ms block while streaming:

```
{"type":"data","seq":..,"t":<ms>,"i":..,"q":..,"amp":<fraction of FS peak>,"amp_db":..,"ph":<deg>,
 "rms":..,"rms_db":..,"peak":<16-bit>,"ovl":..,"clk":..,"freq":..,"light":..,"gain":..,"dc":..,
 "mod_seq":..,"t_us":..}
{"type":"burst","seq":..,"n":4800,"fs":48000,"p_bclk":P,"ph0":<reference phase at sample 0, cycles>,
 "b64":"<4800 int16 little-endian, base64>"}
```

`log` and `error` messages are unsolicited. Boot messages are queued until a
host connects; data and bursts are only produced while a host holds DTR.

## Safety

GP0 is driven low before anything else at boot. The light is switched off
by a timer when no command arrives for `heartbeat` seconds (the main loop
is not involved), when the USB host drops DTR, unmounts or suspends, on
panic or hard fault, and a 1 s watchdog reboots a stuck firmware
(`reset_reason` in `id`, `watchdog` self-test entry).

## Self-test entries

`adc_i2c_ack, adc_register_test, adc_reset_defaults, adc_clock_detect,
adc_active_mode, sysclk, bclk_toggling, fsync_toggling, bclk_per_frame,
block_timing, lockin_math, dma_overruns, usb_tx, watchdog, adc_i2c_errors,
mic_noise_floor, adc_overload, adc_dc_offset, adc_ch2_compare (with ch2),
mod_pin, modulator_state, mod_sync, light_functional (PC side),
selftest_duration`.

`lockin_math` runs the real MAC on a synthetic 24-bit sine (P = 3072, 320,
15360) and requires an amplitude error below 1e-5 and a phase error below
0.001 degrees; `block_timing` checks the 100 ms block period against the
1 MHz timer (same crystal) to +-2 us.

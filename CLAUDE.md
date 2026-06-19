# bibby - Brew In a Bag Temperature Ramp Controller

Embedded temperature/power controller running on Raspberry Pi 5.
Two separate processes communicate via POSIX shared memory (`/biab_heater`).
1) heater-controller controls solid state relays (SSRs)
2) frontend controls GUI, PID controller, temp readings

## Processes

### heater-controller (C) — `heater-controller/main.c`
- Waits for AC zero-crossing rising edge on GPIO 20 (pin 38) via libgpiod v2 edge events
- Drives two SSRs: GPIO 21 (pin 40) = SSR1, GPIO 26 (pin 37) = SSR2
- Sigma-delta duty cycle: accumulates duty fraction each ZC, fires SSR when accumulator ≥ 1.0; accumulator capped at 2.0
- Watchdog: mains-derived timeout with no ZC (half-period + 0.7ms guard; ~9ms at 60Hz, ~10.7ms at 50Hz) → turns off both SSRs, sets `watchdog_alarm`
- `simulate_zc` flag in shm lets frontend fake zero crossings for bench testing
- GPIO chip: `/dev/gpiochip4` (Pi 5)

### frontend (C++) — `frontend/main.cpp`
- SDL3 + Dear ImGui + GLES3
- Physical display is mounted sideways: ImGui renders into a portrait-sized FBO, then a fullscreen quad blits it to the landscape screen rotated 90° CW via a custom GLSL shader
- Touch input X/Y coordinates are remapped to match the rotation before being fed to ImGui
- UI: temperature readout, SSR1/SSR2 on/off state + duty%, DRDY pin state, watchdog alarm, "Test Touch" button (toggles simulate_zc), two vertical duty cycle sliders
- duty1/duty2 written to shm each frame; output1/output2/watchdog_alarm read from shm each frame
- Implements a PID controller on the temperature to control the duty cycle of the SSRs.
- Logs one CSV row per fresh temperature sample (see CSV Logging below).

## CSV Logging — `frontend/csv_logger.{h,cpp}`
- `CsvLogger` opens one timestamped file per run on construction: `~/bibby/logs/YYYY/MM/DD/HH-MM-SS.csv` (directories auto-created). Logging is continuous for the life of the frontend.
- One row per fresh sensor sample, written from `main.cpp` right after `pid.update()` so the PID terms are current. Variable `dt` — recover time from the `wall_time` (ISO-8601 local, ms) or `t_monotonic_s` columns.
- Each row is `fflush`ed so a crash mid-brew keeps the data on disk.
- `PidController::terms()` returns a `PidTerms` (residual `error`, individual `p`/`i`/`d` contributions, raw `integral`/`deriv` state, clamped `output`) for logging/tuning.
- Columns: `wall_time, t_monotonic_s, temp_raw_c, temp_filt_c, setpoint_c, duty1, duty2, pid_output, pid_error, pid_p, pid_i, pid_d, pid_integral, pid_deriv, manual, grain_in, rtd_fault, watchdog`.

## MAX31865 Sensor — `frontend/sensors/max31865.{h,cpp}`
- PT100, 3-wire, 400Ω reference resistor
- SPI: `/dev/spidev0.0`, 1 MHz, SPI_MODE_1
- Constructor sequence: set 3-wire + fault-clear + mains filter notch → enable Vbias (10ms wait) → enable auto-conversion → discard first 10 samples using DRDY polling
- Mains filter: config reg bit 0 selects the noise-rejection notch (set = 50 Hz, clear = 60 Hz); the `mains_hz` ctor arg (from `bibby.ini`) sets it. Must be chosen before auto-conversion is enabled (datasheet)
- `read_temperature()`: non-blocking — checks DRDY level, returns `last_temp_read_` if not ready, otherwise reads RTD registers and computes °C via Callendar-Van Dusen
- DRDY pin: GPIO 16 (pin 36), active-low, falling-edge detection; `gpiod_line_request*` passed into constructor and stored as member

## Shared Memory — `shared/shm_types.h`
| Field | Writer | Type | Purpose |
|---|---|---|---|
| `duty1`, `duty2` | frontend | float [0,1] | SSR duty cycles |
| `simulate_zc` | frontend | bool | fake ZC for testing |
| `frontend_iteration` | frontend | uint32 | incremented each UI frame; heater-controller forces duties to zero if stale for ~2s (2×mains_Hz ZC cycles) |
| `output1`, `output2` | heater-controller | bool | current SSR fired state |
| `watchdog_alarm` | heater-controller | bool | no ZC within the mains-derived timeout |
| `iteration` | heater-controller | uint32 | ZC counter |

## Pin assignments (40-pin header)
| Pin | GPIO | Signal | Direction | Used by |
|---|---|---|---|---|
| 1 | — | 3.3V | Power | MAX31865 VIN |
| 6 | — | GND | Power | MAX31865 GND |
| 19 | GPIO10 | SPI0 MOSI | Out | MAX31865 SDI |
| 21 | GPIO9 | SPI0 MISO | In | MAX31865 SDO |
| 23 | GPIO11 | SPI0 SCLK | Out | MAX31865 CLK |
| 24 | GPIO8 | SPI0 CS | Out | MAX31865 CS |
| 36 | GPIO16 | DRDY | In | MAX31865 DRDY (frontend) |
| 37 | GPIO26 | SSR2 | Out | heater-controller |
| 38 | GPIO20 | Zero Cross | In | heater-controller |
| 40 | GPIO21 | SSR1 | Out | heater-controller |

## Configuration — `shared/config.{h,c}` + `bibby.ini`
- `bibby.ini` (repo root) holds the parameters another person/setup would change. Both processes load the same file at startup via `config_load()` (plain C, compiled into both targets; `extern "C"` so the C++ frontend links it).
- Lookup order: `path` arg → `$BIBBY_CONFIG` env var → compile-time `BIBBY_CONFIG` define (baked to the source-tree `bibby.ini`, like the `BIBBY_PATH` heater-binary define; not yet relocatable for install). The env var and the compile-time default share the `BIBBY_CONFIG` name by design. Missing file or unknown keys → built-in defaults (60 Hz, PID gains zero).
- INI format: `[section]` headers, `key = value`, `#`/`;` comments (inline too). Keys: `mains.frequency_hz`, `element1/2.watts`, `element1/2.area_cm2`, `pid.kp/ki/kd`, `sensor.ref_resistor_ohms`, `sensor.temp_cal_gain`, `sensor.temp_cal_offset`.
- `mains.frequency_hz` (50/60) drives the heater-controller's timing: `config_zc_timeout_ns()` = mains half-period + 0.7 ms guard (60 Hz→9.0 ms, 50 Hz→10.7 ms) for the ZC watchdog/sim period; `config_frontend_watchdog_zc()` = `2*hz` (~2 s) for the frontend-stale cutoff. It also sets the MAX31865 mains filter notch (passed to the sensor ctor; see above).
- `pid.kp/ki/kd` are passed to the `PidController` constructor in `main.cpp`.
- `sensor.ref_resistor_ohms`/`temp_cal_gain`/`temp_cal_offset` are the per-unit RTD calibration (ice-point Rref trim + two-point span fit), passed to the `Max31865` ctor in `main.cpp`. Identity defaults (400 Ω / 1.0 / 0.0) = uncalibrated. Derivation formulas live in `bibby.ini` and README §9.
- `element*.watts`/`element*.area_cm2` are loaded and held in `BibbyConfig`; reserved for the planned watts→duty / power-density control law (see memory [[pid-units-watts-to-duty]]), not yet consumed by control.

## Build
CMake. SDL3 via FetchContent. ImGui vendored in `third_party/imgui`.
Frontend links: SDL3-static, GLESv2, gpiod, rt. `shared/config.c` is compiled into both targets.
`build/` and `logs/` are gitignored. `bibby.ini` is checked in.

## Safety
- A  safe and robust system is important as this is dealing with high power and high voltage.
- The heater-controller needs to be up at all times. When it is exiting, crashes, or starts up, output to the SSR's shall be low and duty% commands be zero. Don't worry about heater-controller starting up when the raspberry pi boots up, I will figure that out. When the frontend runs, and does not detect the heater-controller running, it will load up the heater-controller.
- If the heater-controller does not detect the frontend, running and servicing the shared memory, the outputs to the SSRs shall be low (setting duty% to zero suffices for this requirement).
- When the frontend does not have reliable temperature, i.e. when a fault occurs, only manual control is allowed to drive the SSRs.

## Configurability
- There shall be an ini file that configures key parameters that another person/setp would need.
- Items for configuration include, 50Hz/60Hz, wattage per element, surface area of element per element, PID parameters.
- All configurable items should be in the ini file. There should not be a circumstance for someone to have to change a constant in the code and have to recompile to use.


## Documentation
- This project should be documented in a way that someone else can reproduce this project.
- The read me for this project should include everything, including the enclosure, electrical diagram, raspberry pi hat PCB, bring up and installation of raspberry pi softare, and methodology of calibrating control parameters for a future brewing set up.
- The documentation should include diagrams and architecture of how the software works.
- As updates are made, ensure that the readme has up-to-date information and includes no missing features. The readme should have a good flow for someone not familiar with the project to become familiar, good structure in documentation.
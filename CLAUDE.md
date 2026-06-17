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
- Watchdog: 9-millisecond timeout with no ZC → turns off both SSRs, sets `watchdog_alarm`
- `simulate_zc` flag in shm lets frontend fake zero crossings for bench testing
- GPIO chip: `/dev/gpiochip4` (Pi 5)

### frontend (C++) — `frontend/main.cpp`
- SDL3 + Dear ImGui + GLES3
- Physical display is mounted sideways: ImGui renders into a portrait-sized FBO, then a fullscreen quad blits it to the landscape screen rotated 90° CW via a custom GLSL shader
- Touch input X/Y coordinates are remapped to match the rotation before being fed to ImGui
- UI: temperature readout, SSR1/SSR2 on/off state + duty%, DRDY pin state, watchdog alarm, "Test Touch" button (toggles simulate_zc), two vertical duty cycle sliders
- duty1/duty2 written to shm each frame; output1/output2/watchdog_alarm read from shm each frame

## MAX31865 Sensor — `frontend/sensors/max31865.{h,cpp}`
- PT100, 3-wire, 400Ω reference resistor
- SPI: `/dev/spidev0.0`, 1 MHz, SPI_MODE_1
- Constructor sequence: set 3-wire + fault-clear → enable Vbias (10ms wait) → enable auto-conversion → discard first 10 samples using DRDY polling
- `read_temperature()`: non-blocking — checks DRDY level, returns `last_temp_read_` if not ready, otherwise reads RTD registers and computes °C via Callendar-Van Dusen
- DRDY pin: GPIO 16 (pin 36), active-low, falling-edge detection; `gpiod_line_request*` passed into constructor and stored as member

## Shared Memory — `shared/shm_types.h`
| Field | Writer | Type | Purpose |
|---|---|---|---|
| `duty1`, `duty2` | frontend | float [0,1] | SSR duty cycles |
| `simulate_zc` | frontend | bool | fake ZC for testing |
| `frontend_iteration` | frontend | uint32 | incremented each UI frame; heater-controller forces duties to zero if stale for ~2s (120 ZC cycles) |
| `output1`, `output2` | heater-controller | bool | current SSR fired state |
| `watchdog_alarm` | heater-controller | bool | no ZC within 9s |
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

## Build
CMake. SDL3 via FetchContent. ImGui vendored in `third_party/imgui`.
Frontend links: SDL3-static, GLESv2, gpiod, rt.
`build/` is gitignored.

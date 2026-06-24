# bibby — Brew In A Bag Temperature Ramp Controller

`bibby` is a temperature and power controller for electric brew-in-a-bag (BIAB)
brewing, running on a Raspberry Pi 5. It reads kettle temperature from a PT100
RTD, drives two mains heating elements through solid-state relays (SSRs), and
presents a touch UI for setting and tracking the brew temperature. Power is
delivered as a duty cycle synchronized to the AC mains zero crossing, and the
whole system is built so that any fault, crash, or loss of a process leaves the
heaters **off**.

This README is meant to let someone unfamiliar with the project understand it,
reproduce the hardware, build the software, and calibrate it for their own
kettle. It moves from the big picture down to the details: architecture, then
hardware, then software, then build/bring-up, then calibration.

This project is for fun. I am doing it to improve my home brewing set up. I hope that someone sees inspiration in this project and does something similar. I am doing this with a couple specific goals mind
- Learn to use AI for coding purposes.
- I recently purchased a Mac mini. I will only use a Mac for this project and it forces me to learn how to use it
- I've recently acquired a 3D printer. This project has one 3D printed part, so I should learn how to use it.

---

## Table of contents

1. [System overview](#1-system-overview)
2. [Software architecture](#2-software-architecture)
3. [Safety design](#3-safety-design)
4. [Hardware](#4-hardware)
5. [Raspberry Pi bring-up and installation](#5-raspberry-pi-bring-up-and-installation)
6. [Building bibby](#6-building-bibby)
7. [Configuration](#7-configuration)
8. [Running](#8-running)
9. [Calibration methodology](#9-calibration-methodology)
10. [Data logging](#10-data-logging)
11. [Repository layout](#11-repository-layout)
12. [License](#12-license)

---

## 1. System overview

The controller is split into **two cooperating processes** that talk through a
single POSIX shared-memory segment (`/biab_heater`):

```
  +----------------------------------------------------------------+
  |                      Raspberry Pi 5                             |
  |                                                                |
  |   +---------------------+        +-------------------------+   |
  |   |     frontend        |        |    heater-controller    |   |
  |   |     (C++)           |        |    (C)                  |   |
  |   |                     | shm    |                         |   |
  |   |  - SDL3 + ImGui UI  |<------->|  - zero-cross capture   |   |
  |   |  - MAX31865 (SPI)   | /biab_ |  - sigma-delta SSR fire  |   |
  |   |  - temp filter      | heater |  - watchdogs            |   |
  |   |  - PID controller   |        |                         |   |
  |   |  - CSV logging      |        |                         |   |
  |   +----------+----------+        +-----+-------------+-----+   |
  |              |                         ^             |         |
  +--------------|-------------------------|-------------|---------+
                 | SPI0 + DRDY             | GPIO20 ZC   | GPIO21/26
                 v                         |             v
          +-------------+          +-------------+   +-----------+
          |  MAX31865   |          | zero-cross  |   |   SSR1    |
          |  + PT100    |          | detector    |   |   SSR2    |
          +-------------+          +-------------+   +-----------+
                 ^                        ^               |
                 | RTD                    | AC mains      | switched mains
              kettle probe                line sense      to heating elements
```

- **heater-controller** owns the relays. It must be up at all times; the
  frontend launches it if it is not already running.
- **frontend** owns the sensor, the control law (PID), the UI, and logging. It
  never touches the relays directly — it only writes duty-cycle commands into
  shared memory.

This separation keeps the safety-critical, real-time relay logic small, simple,
and independent of the much larger GUI process.

### Why a duty cycle synchronized to zero crossing?

The heating elements are resistive loads switched by zero-cross SSRs. Switching
only at the AC zero crossing eliminates the inrush/EMI of phase-angle control.
Power is modulated by choosing *which* mains half-cycles to pass — a duty cycle
in `[0, 1]`. A sigma-delta modulator (below) spreads the fired cycles out evenly
so the average power tracks the command with minimal low-frequency flicker.

---

## 2. Software architecture

### 2.1 Shared memory — `shared/shm_types.h`

A single `HeaterShm` struct mapped by both processes. Every field is a C11/C++
`atomic` so the two processes can read and write without locks. The segment is
created by the heater-controller (`O_CREAT`) and is **never** `shm_unlink`ed, so
a restarted controller reattaches to the same segment instead of silently
creating a new, separate one.

| Field | Writer | Type | Purpose |
|---|---|---|---|
| `duty1`, `duty2` | frontend | float [0,1] | SSR1 / SSR2 duty cycles |
| `simulate_zc` | frontend | bool | fake zero crossings for bench testing |
| `frontend_iteration` | frontend | uint32 | incremented each UI frame; a heartbeat |
| `output1`, `output2` | heater-controller | bool | current fired state of each SSR |
| `watchdog_alarm` | heater-controller | bool | set when no ZC seen within timeout |
| `iteration` | heater-controller | uint32 | ZC counter; the frontend uses it to detect a live controller |

### 2.2 heater-controller (C) — `heater-controller/main.c`

A single-threaded loop pinned to the AC mains zero crossing:

1. **Single instance.** `flock` on `/tmp/biab_heater.lock`; a second instance
   exits immediately. The lock is released automatically by the kernel on exit
   or crash.
2. **Safe startup.** Open/create the shm segment and force `duty1=duty2=0`,
   `output1=output2=false`, alarms cleared — so a restart never re-fires the
   relays at a stale duty.
3. **Zero-cross capture.** Wait for a rising edge on GPIO20 using libgpiod v2
   edge events, with a timeout derived from the configured mains frequency (§7;
   9.0 ms at 60 Hz, 10.7 ms at 50 Hz).
4. **Sigma-delta modulation.** Each zero crossing, add the duty fraction to a
   per-channel accumulator; when it reaches 1.0, fire that SSR for the half
   cycle and subtract 1.0. The accumulator is capped at 2.0 to bound catch-up.
   This is a first-order delta-sigma DAC clocked by the mains: the average fired
   fraction equals the commanded duty, with the switching energy pushed to high
   frequency.
5. **Two watchdogs:**
   - **Zero-cross watchdog:** if no ZC arrives within the timeout above, both
     SSRs are forced off and `watchdog_alarm` is set (a lost mains-sense signal
     must not leave a relay latched on). The timeout follows the configured
     mains frequency automatically.
   - **Frontend heartbeat watchdog:** if `frontend_iteration` does not advance
     for `2 × mains_Hz` zero crossings (~2 s; 120 at 60 Hz, 100 at 50 Hz),
     duties are forced to zero — a dead frontend must not leave the heaters
     running.
6. **Safe shutdown.** On SIGINT/SIGTERM, drive both SSRs low before exiting.

`simulate_zc` lets the frontend (via the "Test Touch" button) inject fake zero
crossings so the UI and modulator can be exercised on the bench with no mains
connected.

### 2.3 frontend (C++) — `frontend/main.cpp`

SDL3 + Dear ImGui on GLES3. Per-frame loop:

1. **Ensure the controller is up.** On startup, probe the shm `iteration`
   counter; if the segment is missing or the counter is not advancing, launch
   the heater-controller (double-fork + `setsid` so it is reparented to init and
   outlives the frontend), then wait for the segment to appear.
2. **Read the sensor** (non-blocking; see §2.4). Fresh samples are smoothed and
   pushed to the PID and the on-screen history graph.
3. **Choose control mode:**
   - **Manual:** the on-screen slider sets the duty for both SSRs.
   - **Auto:** the PID output drives both SSRs.
   - If the temperature is **not reliable** (any RTD fault), the code forces
     manual mode so the PID can never command power on bad data.
4. **Publish** `duty1`, `duty2`, `simulate_zc`, and bump `frontend_iteration`.
5. **Log** one CSV row per fresh sample (§10).

#### Rotated display — `frontend/display.{h,cpp}`

The physical panel is mounted sideways. The UI is rendered into a
**portrait-sized off-screen framebuffer**, then a fullscreen quad blits it to the
landscape screen **rotated 90° CW** via a small GLSL shader. Touch coordinates
are remapped through the same rotation before being fed to ImGui, so touches land
where they appear.

#### Temperature filter — `frontend/temp_filter.{h,cpp}`

`SecondOrderAverage`: two cascaded boxcar (moving-average) stages — a
second-order CIC with triangular weighting — over a 40-sample window. The first
sample primes both stages so the output starts at the true value instead of
ramping from zero.

#### PID controller — `frontend/pid.{h,cpp}`

`PidController::update(temp_c, setpoint_c, feedforward)` runs once per fresh
sample and returns a power command `clamp(feedforward + p + i + d)` in `[0, 1]`.
The full scaffolding is in place — per-sample execution, output clamp to
`[0,1]`, conditional-integration anti-windup, and `reset()` — but the gains come
from `bibby.ini` (§7) and **default to zero**, so the feedback commands zero
power until the loop is tuned (see §9.5). The optional `feedforward` argument is
the holding duty (§9.6); it defaults to zero. `terms()` exposes the
feedforward / P / I / D breakdown and internal state for logging and tuning.

> Note: the loop currently treats one sample as one time step (`dt` folded into
> the gains). If the sample cadence changes, make `dt` explicit.

### 2.4 MAX31865 RTD sensor — `frontend/sensors/max31865.{h,cpp}`

- PT100, **3-wire**, 400 Ω nominal reference resistor, on `/dev/spidev0.0` at
  1 MHz, `SPI_MODE_1`.
- **Constructor sequence:** set 3-wire + clear faults + mains filter notch →
  enable VBIAS (10 ms settle) → enable auto-conversion (20 ms) → discard the
  first conversions using DRDY polling.
- **Mains filter:** config-register bit 0 selects the noise-rejection notch
  (set = 50 Hz, clear = 60 Hz). It is set from `mains.frequency_hz` (§7) and
  written in the first config word — before auto-conversion, as the datasheet
  requires — so the sensor rejects local mains pickup.
- **`read_temperature()` is non-blocking:** it checks the DRDY pin (GPIO16,
  active-low). If a conversion is not ready it returns the previous value with
  `is_fresh=false`; otherwise it burst-reads RTD + fault registers, converts via
  the Callendar–Van Dusen equation, and applies the per-unit gain/offset
  calibration (§9.2).
- **Fault handling:** the RTD-LSB fault bit and the Fault Status register (07h)
  are checked every read. A faulted conversion is rejected (the stale value is
  held so the filter is not poisoned) and the fault latch is cleared so the
  status reflects the next conversion. `decode_fault_status()` turns the register
  into human-readable text shown on the UI.

The MAX31865 datasheet is included at [`docs/MAX31865.pdf`](docs/MAX31865.pdf).

---

## 3. Safety design

This system switches mains voltage into multi-kilowatt heating elements, so the
**default state is always off**. The design requirements and how they are met:

| Requirement | Mechanism |
|---|---|
| Heaters off on controller startup | shm duties forced to 0 before the relay loop starts |
| Heaters off on controller exit/crash | SIGINT/SIGTERM handler drives SSRs low; `flock` released by kernel on crash |
| Only one controller / one frontend at a time | `flock` single-instance locks (`shared/single_instance.h`) |
| Heaters off if the mains-sense (ZC) signal is lost | mains-derived zero-cross watchdog (~9 ms at 60 Hz) forces SSRs off, raises `watchdog_alarm` |
| Heaters off if the frontend stops servicing the loop | frontend heartbeat watchdog (~2 s) forces duties to zero |
| The controller is always available to the frontend | frontend auto-launches the controller (detached) if it is not running |
| No power commanded on unreliable temperature | an RTD fault forces manual mode; the PID path cannot drive the relays |
| Duty commands always bounded | duties clamped to `[0,1]` in both processes |

> The heater-controller is expected to also be started at boot by the operating
> system; that is a deployment detail left to the integrator. The frontend's
> auto-launch is a backstop, not the primary mechanism.

---

## 4. Hardware

### 4.1 Major components

| Item | Notes |
|---|---|
| Raspberry Pi 5 | GPIO chip is `/dev/gpiochip4` on Pi 5 |
| Touchscreen display | Mounted in portrait; software rotates the UI 90° CW |
| PT100 RTD probe | 3-wire, immersed in the kettle |
| 2 × zero-cross SSR | One per heating element; switched by GPIO21 / GPIO26 |
| 2 × resistive heating element | Mains-powered, BIAB kettle |
| Custom Pi HAT PCB | Sensor front-end + zero-cross detector (§4.2) |

> **TODO (fill in your build):** exact display model, SSR part numbers and
> current rating, heating-element wattage, fusing/contactor, and mains
> connector. These are deployment-specific and not captured in the repo.

### 4.2 Raspberry Pi HAT PCB — `pi_hat/bibby_pi_hat/`

A KiCad 7+ project (`.kicad_sch`, `.kicad_pcb`, `.kicad_pro`). Open it in KiCad
to view/edit the schematic and board, or to regenerate gerbers
(`pi_hat/bibby_pi_hat/gerbers/`). The HAT carries two functional blocks:

**RTD front-end (U1 — MAX31865):**
- `R1` = 400 Ω 0.1 % — RTD reference resistor (`Rref`). Precision and tempco here
  set the measurement accuracy; this is one of the two per-unit calibration
  anchors (§9.1).
- `C1`, `C3` = 0.1 µF — decoupling / RC filtering.
- SSOP-20 package; SPI to the Pi plus the DRDY data-ready line.

**AC zero-cross detector (U2 — H11AA1 AC-input optocoupler):**
- The H11AA1 has an anti-parallel LED input, so it conducts on both mains
  half-cycles and its phototransistor output pulses around each zero crossing.
- `R3`–`R6` = 43 kΩ, `R2` = 20 kΩ, `R7` = 1 MΩ (! DNI !) — line current-limiting and
  pull/​bias network for the optocoupler. The pulse train feeds GPIO20.
- Solder jumpers `JP1`/`JP2`/`JP3` select options on the board.

**Connectors:**
- `J1` — 2×20 stacking header to the Pi GPIO.
- `J2` — Phoenix 1×10 5.08 mm terminal block (field wiring: RTD, mains sense,
  SSR drive).

> The complete, authoritative bill of materials and net list is the KiCad
> schematic itself. The values above are transcribed from it for orientation.

### 4.3 Pin assignments (40-pin header)

| Pin | GPIO | Signal | Direction | Used by |
|---|---|---|---|---|
| 1 | — | 3.3 V | Power | MAX31865 VIN |
| 6 | — | GND | Power | MAX31865 GND |
| 19 | GPIO10 | SPI0 MOSI | Out | MAX31865 SDI |
| 21 | GPIO9 | SPI0 MISO | In | MAX31865 SDO |
| 23 | GPIO11 | SPI0 SCLK | Out | MAX31865 CLK |
| 24 | GPIO8 | SPI0 CE0 | Out | MAX31865 CS |
| 36 | GPIO16 | DRDY | In | MAX31865 DRDY (frontend) |
| 37 | GPIO26 | SSR2 | Out | heater-controller |
| 38 | GPIO20 | Zero Cross | In | heater-controller |
| 40 | GPIO21 | SSR1 | Out | heater-controller |

### 4.4 Electrical / wiring overview

```
   MAINS L ---+----------------------------+--------------------+
              |                            |                    |
              |                       [SSR1 load]          [SSR2 load]
              |                            |                    |
        [ZC detector U2]             [Element 1]          [Element 2]
              |  (opto, isolated)          |                    |
              v                       [SSR1 line]          [SSR2 line]
        GPIO20 (ZC in)                     ^                    ^
                                           |                    |
   Pi 3.3V/GPIO control side:        GPIO21 (SSR1)        GPIO26 (SSR2)
                                      drive (3.3V logic into SSR input)

   PT100 RTD --3 wires--> MAX31865 (U1) --SPI0 + DRDY--> Pi
```

- The H11AA1 provides **galvanic isolation** between the mains line-sense and the
  Pi logic.
- The SSRs are driven by 3.3 V GPIO on their input side; their output side
  switches mains into the elements.
- The RTD is a low-voltage measurement isolated from the mains by the kettle and
  the sensor front-end.

> **WARNING:** mains wiring, fusing, grounding/earth bonding, and enclosure
> safety are the integrator's responsibility. Use appropriately rated SSRs and
> heat-sinking, a proper earth connection to the kettle, and a contactor/fuse/breaker
> sized to the elements.

### 4.5 Enclosure

> **TODO (fill in your build):** enclosure material and dimensions, panel cutout
> for the display, ventilation/heat-sinking for the SSRs, and how the Pi + HAT +
> SSRs are mounted. Add photos and mechanical drawings here.

---

## 5. Raspberry Pi bring-up and installation

Tested target: **Raspberry Pi 5**, 64-bit Raspberry Pi OS.

### 5.1 OS and interfaces

1. Flash Raspberry Pi OS (64-bit) and boot the Pi. Install a beefy SD card (or other) for logging. Find a way to remote login to the pi.
2. Enable SPI:
   ```
   sudo raspi-config   # Interface Options -> SPI -> Enable
   ```
   Confirm `/dev/spidev0.0` exists after reboot.
   
   The Pi 5 exposes GPIO via `/dev/gpiochip4`; both programs open it directly.
   The user running the binaries must have access to the GPIO and SPI devices
   (the `gpio` and `spi` groups, or run via a systemd unit with the right
   permissions).

### 5.2 Build dependencies

```
sudo apt update
sudo apt install -y build-essential cmake git \
                    libgpiod-dev \
                    libgles2-mesa-dev \
                    libgl1-mesa-dev
```

- **libgpiod v2** is required (the code uses the v2 edge-event API). Confirm with
  `gpiodetect` / `pkg-config --modversion libgpiod`.
- **SDL3** is fetched and built from source by CMake (FetchContent), so it does
  not need to be installed. Its build may pull additional dev packages
  (e.g. `libinput`, `libudev`, X/Wayland or KMS/DRM headers) depending on how
  you run the display; install what SDL's configure step reports as missing.

### 5.3 Autostart at boot (systemd)

The frontend is the only process that needs to be started — it auto-launches
the heater-controller if it is not already running. Create a systemd service
to bring up the frontend on boot.

Create `/etc/systemd/system/bibby.service`:

```ini
[Unit]
Description=bibby brew controller frontend
After=multi-user.target

[Service]
User=ramp
SupplementaryGroups=gpio spi video render input
Environment=BIBBY_CONFIG=/home/ramp/bibby/bibby.ini
ExecStart=/home/ramp/bibby/build/frontend/frontend
Restart=on-failure
RestartSec=3

[Install]
WantedBy=multi-user.target
```

Adjust `User` and `ExecStart` to match your username and the actual path to
the built binary (see §6). Enable and start it:

```
sudo systemctl daemon-reload
sudo systemctl enable bibby
sudo systemctl start bibby
```

Check status and logs with:

```
systemctl status bibby
journalctl -u bibby -f
```

The heater-controller inherits the service's group membership, so it also
has the GPIO access it needs. If either process is killed, the heater-controller
always drives the SSRs low before exiting; the frontend's watchdog in shared
memory likewise forces duty to zero if the frontend stops updating.

---

## 6. Building bibby

Clone the repository first:

```
git clone --recurse-submodules <repo-url> bibby
cd bibby
# if you already cloned without submodules:
git submodule update --init --recursive
```

Dear ImGui is vendored as the `third_party/imgui` submodule.

Standard CMake out-of-source build:

```
cmake -S . -B build -DCMAKE_BUILD_TYPE=Release
cmake --build build -j
```

This produces two binaries:

- `build/heater-controller/heater-controller`
- `build/frontend/frontend`

Notes:

- The frontend links `SDL3-static`, `GLESv2`, `gpiod`, and `rt`. The
  heater-controller links `gpiod` and `rt`.
- The frontend embeds the build-tree path of the heater-controller binary
  (`BIBBY_PATH`) so it can auto-launch the freshly built controller
  during development. **This path is not yet relocatable** — there is no
  `make install` rule, and the path would need to be switched to runtime
  resolution before installing to a target. `build/` and `logs/` are gitignored.

---

## 7. Configuration

Runtime parameters that depend on the install (mains region, heating elements,
control gains) live in **`bibby.ini`** at the repo root, **not** in the source.
Both processes read the same file on startup, so a new brew setup is adapted by
editing one file — no rebuild required.

```ini
[mains]
frequency_hz = 60        ; 60 = North America, 50 = most elsewhere

[element1]               ; SSR1, GPIO21 / pin 40
watts    = 2500          ; rated electrical power, W
area_cm2 = 150           ; wetted surface area, cm^2

[element2]               ; SSR2, GPIO26 / pin 37
watts    = 2500
area_cm2 = 150

[pid]                    ; temperature-loop gains; output is duty in [0,1]
kp = 0.0
ki = 0.0
kd = 0.0

[feedforward]            ; holding-duty feedforward (§9.6); 0 gain = disabled
process_gain_c = 0.0     ; identified process gain K, °C per unit duty (§9.4)
ambient_c      = 20.0    ; cold-start / room reference temperature, °C

[sensor]                 ; per-unit RTD calibration (re-derive per probe, §9)
ref_resistor_ohms = 397.82   ; MAX31865 Rref, ice-point trimmed (§9.1)
temp_cal_gain     = 1.0528   ; two-point span gain  (§9.2)
temp_cal_offset   = -0.032   ; two-point span offset, °C (§9.2)
```

How it is loaded (`shared/config.{h,c}`, compiled into both binaries):

- **Lookup order:** the `$BIBBY_CONFIG` environment variable, then the
  compile-time default (the `BIBBY_CONFIG` define, the source-tree `bibby.ini`).
  Set `$BIBBY_CONFIG` to point a deployed binary at an installed config. (The
  env var and the compile-time default deliberately share the `BIBBY_CONFIG`
  name.)
- **Robust by default:** a missing file or unknown key falls back to a built-in
  default (60 Hz, zero PID gains), so the system still comes up safely. Comments
  (`#` or `;`, inline allowed) and blank lines are ignored.

What each parameter drives:

| Key | Used by | Effect |
|---|---|---|
| `mains.frequency_hz` | both | heater-controller: zero-cross watchdog timeout = mains half-period + 0.7 ms guard (60 Hz → 9.0 ms, 50 Hz → 10.7 ms), and the frontend-stale cutoff (`2 × Hz` ≈ 2 s of zero crossings). frontend: selects the MAX31865 noise-rejection filter notch (50/60 Hz) written at sensor startup. |
| `pid.kp` / `ki` / `kd` | frontend | PID gains handed to the controller at construction (§9.5). |
| `feedforward.process_gain_c` / `ambient_c` | frontend | Holding-feedforward gain and reference; `u_ff = (setpoint − ambient_c)/process_gain_c`. A zero gain disables feedforward (§9.6). |
| `sensor.ref_resistor_ohms` | frontend | MAX31865 reference resistor `Rref`, trimmed at the ice point; scales the RTD resistance reading (§9.1). |
| `sensor.temp_cal_gain` / `temp_cal_offset` | frontend | Two-point span correction applied to the temperature, `T_true = gain·T + offset` (§9.2). |
| `element*.watts` / `area_cm2` | frontend | Loaded into the config struct; reserved for the planned watts→duty / power-density control law — not yet consumed by the loop. |

> Every per-unit calibration constant lives in `bibby.ini`; none require a code
> change or rebuild. Identity defaults (`Rref` = 400 Ω, gain = 1.0, offset = 0.0)
> leave the sensor uncalibrated but functional. Each value's derivation formula
> is in §9.

> Because the default path is baked to the source tree (like the other
> build-tree paths, §6), packaging for a target should set `$BIBBY_CONFIG` or
> rebuild with the `BIBBY_CONFIG` define pointed at the installed location.

---

## 8. Running

The frontend will start the controller for you:

```
./build/frontend/frontend
```

On launch it checks whether a live heater-controller is present (via the shm
`iteration` heartbeat) and spawns one detached if not. To run the controller on
its own (e.g. for a boot-time service):

```
./build/heater-controller/heater-controller
```

**Bench testing without mains:** press **Test Touch** in the UI to set
`simulate_zc`, which makes the controller treat its mains-derived timeout (§7)
as a zero crossing. The sigma-delta modulator and UI then run with no AC connected.

### UI elements

- Large kettle readout: set point (°C), measured °C and °F, animated heating
  elements whose glow tracks the SSR fired state.
- Full-height **duty slider** (active in manual mode; commands both SSRs).
- **Set-point steppers:** ±10 / ±1 / ±0.1 °C.
- **Toggles:** Test Touch (simulate ZC), Grain In (logged event marker), Manual
  Control (manual vs PID auto).
- **Temperature graph:** last 3 minutes.
- **Fault band:** RTD fault (decoded) and watchdog alarm, when active.

---

## 9. Calibration methodology

Three of the measurement constants are **per-unit** — they depend on the
specific RTD probe and the board's reference resistor, so they **must be
re-derived for any other board or probe**. All three live in the `[sensor]`
section of `bibby.ini`; **none require a code change or rebuild**. Their
identity defaults (`Rref` = 400 Ω, gain = 1.0, offset = 0.0) give a working but
uncalibrated sensor. The procedure below derives them from physical references.

| `bibby.ini` key | Symbol | What it corrects |
|---|---|---|
| `sensor.ref_resistor_ohms` | `Rref` | one-point offset at the ice point (§9.1) |
| `sensor.temp_cal_gain` | `g` | span/slope error across temperature (§9.2) |
| `sensor.temp_cal_offset` | `b` | residual offset after the span fit (§9.2) |

The driver applies them in this order, so **calibrate in this order too**:

```
R_rtd  = (raw / 32768) * Rref           # ref_resistor_ohms  (§9.1)
T_cvd  = CVD(R_rtd)                      # Callendar–Van Dusen, fixed
T_true = g * T_cvd + b                   # gain, offset       (§9.2)
```

### 9.1 Reference resistor `Rref` (ice-point one-point trim)

`Rref` scales the entire resistance reading, so trimming it nulls a constant
offset. Trim it first, at a single known low temperature (the ice point is the
cheapest 0 °C reference available).

**Procedure**

1. Set `sensor.temp_cal_gain = 1.0`, `sensor.temp_cal_offset = 0.0` (so the span
   trim does not mask the reading), and `sensor.ref_resistor_ohms` to the board's
   nominal reference (400 Ω here — the value of `R1` on the HAT, §4.2).
2. Submerge the probe in a well-stirred ice-water bath and let it settle.
3. Read the measured temperature `T_ice` (the **raw** column in the CSV log, §10,
   before any span trim).
4. The PT100 resistance the probe *should* present at `T_ice` is, from
   Callendar–Van Dusen with `R0 = 100 Ω`, `A = 3.9083e-3`:
   `R_pt100(T) = 100·(1 + A·T + B·T²)`, which near 0 °C is ≈ `100·(1 + A·T)`.
   Because the reading scales linearly with `Rref`, the trimmed value is:

   ```
   Rref = Rref_nominal · 100.0 / R_pt100(T_ice)
   ```

**Worked example (this unit):** read **+1.4 °C** in the ice bath.
`R_pt100(1.4) = 100·(1 + 3.9083e-3·1.4) ≈ 100.547 Ω`, so
`Rref = 400 · 100.0 / 100.547 = 397.82 Ω` → `sensor.ref_resistor_ohms = 397.82`.

### 9.2 Span trim `gain`/`offset` (two-point line fit)

With `Rref` set, a probe can still read low or high **in proportion to
temperature** (a probe α / self-heating mismatch). That is a slope error, so it
is corrected in the temperature domain by a straight line through **two**
reference points `(T1_ref, T1_meas)` and `(T2_ref, T2_meas)`, where `T_meas` is
the reading **after** the §9.1 trim:

```
gain   = (T2_ref - T1_ref) / (T2_meas - T1_meas)
offset =  T1_ref - gain · T1_meas
```

Pick the two points as far apart as practical and bracketing the brew range for
the best fit. Good choices: the ice point (0 °C) and the local **boiling point**
(altitude-corrected, below). A stirred ambient bath against a trusted reference
thermometer also works for the upper point.

**Boiling-point reference.** Water boils below 100 °C at altitude; approximate
the local boiling point as `T_boil ≈ 100 − 3.4·(h/1000m)` (°C, `h` = elevation),
or read it from a steam table for your barometric pressure, then use that as the
upper `T_ref`.

**Worked example (this unit):** ice point `(0.00, 0.00)` and a fan-mixed ambient
point `(25.00, 23.78)`:
`gain = (25.00 − 0.00)/(23.78 − 0.00) = 1.0513`,
`offset = 0.00 − 1.0513·0.00 = 0.0`. After a small refinement the shipped values
are `sensor.temp_cal_gain = 1.0528`, `sensor.temp_cal_offset = -0.032`.

> **PROVISIONAL:** both points above are ≤ 25 °C. Re-fit using a boiling-point
> upper reference before trusting brew-range temperatures — at 5230 ft the local
> boiling point is **94.7 °C**, and the §9.1-trimmed RTD should read ~90.0 °C
> there. Re-derive both keys for your own probe and altitude.

### 9.3 Process model

The Laplace-domain model of the signal chain from duty command to measured
temperature has three cascaded stages: the kettle thermal plant, the RTD probe
lag, and the software filter.

#### Signal chain

```
  u ──►[ ×P_max ]──► P ──►[ G_plant ]──► T_kettle ──►[ G_probe ]──►[ G_filt ]──► T_meas ──► PID
 (duty)    [W]            K_p/(τ_p s+1)              1/(τ_pr s+1)   2× N-boxcar

           plant heat loss: T_kettle relaxes toward ambient T_amb at rate h·(T_kettle−T_amb)
```

#### Kettle thermal plant

Energy balance of a well-stirred (recirculated) kettle, linearised about an
operating point:

```
m · c_p · dT/dt = P_in − h · (T − T_amb)

G_plant(s) = T(s)/P(s) = K_plant / (τ_plant · s + 1)

  K_plant   = 1/h              [°C/W]    steady-state gain
  τ_plant   = m · c_p / h     [s]       dominant time constant

  m         liquid mass, kg
  c_p       4 186 J/(kg·°C)  for water
  h         total heat-loss coefficient, W/°C
            (conduction through walls + lid, radiation, evaporation)
```

For a 30 L batch in a reasonably insulated kettle `τ_plant` is typically
**20–40 minutes**.  The heat-loss coefficient `h` is small, making `K_plant`
large; their ratio `P_max / (m · c_p)` is the initial ramp rate at full power
and is easy to measure directly from the step test.

#### RTD probe lag

The PT100 probe has its own thermal mass that causes a first-order lag before
it reaches the liquid temperature:

```
G_probe(s) = 1 / (τ_probe · s + 1)

  τ_probe ≈ 5–30 s   (depends on probe mass, immersion depth, flow past tip)
```

#### Software filter

The `SecondOrderAverage` filter consists of two cascaded N = 40 sample
boxcar stages running at the MAX31865 continuous-conversion rate `f_s`
(≈ 60 Hz / 16.7 ms at the 60 Hz notch, 50 Hz / 20 ms at the 50 Hz notch — one
sample per mains cycle).  Each boxcar introduces a group delay of
`(N−1)/(2·f_s) ≈ 0.33 s`, so the two stages together add approximately:

```
Total filter group delay ≈ N / f_s ≈ 0.65 s   (at 60 Hz notch, N = 40)
```

For controller design the filter is approximated as a pure lag equal to its
group delay, absorbed into the effective dead time `L` below.

#### Combined open-loop transfer function

Duty command to measured temperature:

```
G(s) = P_max · G_plant(s) · G_probe(s) · G_filt(s)

     = K_total / ((τ_plant·s+1) · (τ_probe·s+1) · (τ_filt·s+1)²)

  K_total = P_max / h = (watts1 + watts2) / h   [°C per unit duty]
```

Because `τ_plant` dominates, the higher-order poles (probe, filter) are
absorbed into an effective dead time to give the **First-Order Plus Dead Time
(FOPDT) approximation** used for tuning:

```
G_FOPDT(s) = K · e^(−L·s) / (τ · s + 1)

  K   ≈ K_total = (watts1 + watts2) / h     [°C / unit duty]
  τ   ≈ τ_plant                             [s]   dominant time constant
  L   ≈ τ_probe + N / f_s                   [s]   effective dead time
```

The ratio `L/τ` characterises controllability: `L/τ < 0.3` is straightforward
to control; values above 1.0 indicate a sluggish, delay-dominated plant.  For
a typical BIAB kettle, `L/τ` is well below 0.05 — the dominant challenge is
the large time constant, not the dead time.

#### Gain conventions — continuous vs. INI

The bibby PID accumulates raw error counts (`integral_ += error`, not
`+= error·dt`) and computes raw deltas for the derivative (`deriv = e[n]−e[n−1]`,
not divided by `dt`).  The INI gains therefore relate to continuous-time gains
by:

```
kp_ini  = Kp_c                     (no scaling)
ki_ini  = Ki_c · T_s               (T_s = sample period from CSV)
kd_ini  = Kd_c / T_s
```

`T_s` is estimated automatically from the `t_monotonic_s` column of the log.

---

### 9.4 System identification

The parameters `K`, `τ`, and `L` of the FOPDT model are obtained from an
**open-loop step test**: drive a constant manual duty and record the filtered
temperature response.  The Python script `tools/identify_plant.py` fits the
model and computes PID gains automatically.

#### Step-test procedure

The test requires the kettle to be filled and recirculated, but the heater does
not need to be at any particular starting temperature.

1. **Fill** the kettle to the intended brew volume and start the recirculation
   pump.  Good mixing is required for the lumped-thermal-mass model to hold.
2. **Stabilise.** Let the temperature settle for a few minutes with manual duty
   at 0 %.  Note the starting temperature.
3. **Apply the step.** In the bibby UI:
   - Switch to **Manual Control**.
   - Set the duty slider to **30–50 %** (enough power to produce a clear signal
     while staying well clear of boiling for at least 15 minutes).
   - Leave the duty at that value for the duration of the test.
4. **Log.** The CSV logger runs continuously.  Let the brew run until the
   temperature has risen **15–25 °C** above the starting value, or for at least
   **15 minutes** — whichever comes first.
5. **End the test.** Return the duty slider to 0 and switch back to manual.
   Note the wall-clock time so you can locate the step in the CSV log.
6. **Copy the log** from `~/bibby/logs/YYYY/MM/DD/HH-MM-SS.csv` to your
   analysis machine.

> **Practical notes:**
> - Both SSRs are commanded by the same slider in manual mode, so the total
>   power step is `duty × (watts1 + watts2)`.
> - A stable, calm starting temperature is important; avoid running the test
>   immediately after a previous heating phase.
> - If the liquid reaches boiling during the test, stop immediately — the model
>   changes once evaporative cooling becomes significant.

#### Identifying the step window

Open the CSV in a spreadsheet or text editor and note two timestamps from the
`t_monotonic_s` column:

| Landmark | How to find it |
|---|---|
| `t_start` | The sample just before the duty jumps from 0 to the step value |
| `t_end`   | The last sample before you returned the duty to 0 |

Pass these as `--t-start` and `--t-end` (both in seconds from the log start,
i.e., `t_monotonic_s − t_monotonic_s[0]`).

#### Running the identification script

```
# Install dependencies once
pip3 install numpy scipy matplotlib pandas

# Basic run (auto-detects duty from the log)
python3 tools/identify_plant.py logs/YYYY/MM/DD/HH-MM-SS.csv \
        --t-start 120 --t-end 1020

# Specify duty explicitly (more reliable), with λ ≈ L for a moderately fast loop
python3 tools/identify_plant.py logs/YYYY/MM/DD/HH-MM-SS.csv \
        --t-start 120 --t-end 1020 --duty 0.40 --lambda 30

# Save the plot to a file
python3 tools/identify_plant.py logs/YYYY/MM/DD/HH-MM-SS.csv \
        --t-start 120 --t-end 1020 --duty 0.40 -o step_fit.png
```

The script prints a summary and ready-to-paste `[pid]` and `[feedforward]`
blocks (illustrative numbers below):

```
── FOPDT model ────────────────────────────────────────────────
  K   (process gain)  = 73.4  °C per unit duty
  τ   (time constant) = 1823  s  (30.4 min)
  L   (dead time)     = 28.6  s
  L/τ (relative DT)   = 0.0157  (easy to control)
  Fit RMSE            = 0.041  °C

── bibby.ini gains  (dt = 16.7 ms, discrete, gains folded) ──

  ── IMC-PI  (λ=30.0 s)
  [pid]
  kp = 0.423831
  ki = 0.00000388
  kd = 0.00000

── bibby.ini feedforward  (holding-duty feedforward, README §9.6) ──
  [feedforward]
  process_gain_c = 73.40      # identified K, °C per unit duty
  ambient_c      = 21.50      # window start temp; use cold-soak/room temp
```

The plot shows the measured temperature, the FOPDT fit, the duty trace, and
the fit residuals — inspect the residuals to confirm the model fits well.

#### What the model parameters tell you

| Parameter | Physical meaning | Implication |
|---|---|---|
| `K` | Total power / heat-loss coefficient | Large K means the heater dominates; steady state is well above ambient |
| `τ` | Thermal time constant of the full batch | Larger batch = larger τ; the loop can afford slower response |
| `L` | Probe lag + filter delay | Limits how aggressively you can tune; keep the probe well-immersed |
| `L/τ` | Relative dead time | < 0.1 is easy; > 0.5 requires careful detuning |

---

### 9.5 PID tuning

The control law is in place but **un-tuned** — the default gains are zero, so
auto mode commands zero power until you set `pid.kp/ki/kd` in `bibby.ini` (§7).

#### Choosing a tuning rule

The script offers three rules.  All are derived from the FOPDT model fit (§9.4):

| Rule | Characteristic | When to use |
|---|---|---|
| **IMC-PI** | Smooth, no overshoot; λ sets the trade-off between speed and robustness | First choice for BIAB; tune λ to taste |
| **ZN-PID** | Aggressive, ~25 % overshoot | Upper bound; useful to understand the fastest achievable loop |
| **CC-PID** | Balanced, works well at moderate `L/τ` | Good sanity check against IMC |

For a mash process, **IMC-PI** with `λ ≈ L` (the dead time) is a good starting
point.  Overshoot during a mash step costs enzyme activity, so err towards a
larger λ (slower, more conservative).

```bash
# Try IMC-PI with λ = 60 s (more conservative)
python3 tools/identify_plant.py logs/... --rule imc --lambda 60
```

#### Anti-windup

The primary anti-windup is **conditional integration** (`pid.cpp`): the
integrator is held whenever the commanded output is already against a rail
(0 or 1) and the current error would push it further into that rail.  This keeps
the integrator from winding up against a saturated output, and — once a holding
feedforward is configured (§9.6) — confines it to the `[−u_ff, 1−u_ff]` band the
feedforward leaves available.  A hard `±INTEGRAL_MAX` clamp (`pid.cpp`) is a
secondary backstop.

How much integral authority you need depends on whether you use feedforward:

- **With feedforward (§9.6):** the feedforward supplies the steady-state holding
  power, so the integrator only trims model error.  A small authority is fine —
  the script's low-authority message is informational in this case.
- **Without feedforward:** the integrator must supply *all* the holding power, so
  it needs real authority.  If the script reports `ki × INTEGRAL_MAX` below
  ~10 %, raise `INTEGRAL_MAX` in `frontend/pid.cpp` so `ki × INTEGRAL_MAX ≥ 0.5`
  and rebuild, or accept a small steady-state offset held by the proportional
  term alone.

#### Applying gains and validating

1. Copy the `[pid]` block printed by the script into `bibby.ini` (and the
   `[feedforward]` block too, if using feedforward — see §9.6).
2. Restart the frontend (or kill and relaunch to reload the config).
3. Set a **setpoint 5–10 °C above** the current temperature and switch to
   **Auto** mode.  Observe the temperature on the history graph.
4. A well-tuned response rises smoothly and settles with no or minimal
   overshoot.  If it oscillates, increase λ and rerun the script.
5. Log the validation run and inspect `pid_ff`, `pid_p`, `pid_i`, `pid_d`
   columns to confirm the split between feedforward and feedback is sensible,
   the integral is only trimming, and the derivative is not amplifying noise.

The CSV columns `pid_ff`, `pid_p`, `pid_i`, `pid_d`, `pid_integral`,
`pid_deriv`, `pid_error`, and `pid_output` are specifically provided for this
tuning and validation work.

---

### 9.6 Feedforward (optional, recommended)

A kettle spends most of a brew *holding* a temperature against heat loss.  Left
to the PID alone, the integrator has to wind all the way up to supply that
holding power — slow to converge and a source of overshoot.  A **holding
feedforward** supplies it directly instead, leaving the feedback to trim only
the model error.

#### The control law

```
u_ff    = clamp( (setpoint − ambient_c) / K , 0, 1 )     // holding duty
u_total = clamp( u_ff + (P + I + D) , 0, 1 )             // feedforward + feedback
```

`K` is the identified process gain (°C per unit duty, §9.4) and `ambient_c` is a
reference temperature — both live in the `[feedforward]` section of `bibby.ini`.
Because `K` is exactly what `identify_plant.py` prints, the step test that tunes
the PID also sizes the feedforward; the script prints a ready-to-paste
`[feedforward]` block.  Setting `process_gain_c = 0` disables feedforward (the
safe default).

#### Why it does not destabilise or overshoot

- **It does not change loop stability.** `u_ff` depends on the *setpoint*, not
  the *measurement*, so it sits outside the feedback loop.  The closed-loop
  characteristic equation — and therefore stability and the response shape — are
  unchanged; feedforward only adds a known bias (superposition).
- **It does not cause overshoot.** Driving the plant with exactly its
  steady-state holding power produces a monotonic, first-order approach to the
  setpoint (no overshoot), just slow; the feedback adds the transient push.
  Feedforward *reduces* overshoot versus an integrator that must wind up from
  zero.  The only way feedforward overshoots is if it is **over-sized** — e.g.
  `K` under-estimated, so `u_ff` exceeds the true holding duty and the kettle
  settles above setpoint until the integrator pulls it back.  Keep `K` honest
  (use the identified value); if anything, a slight under-estimate is the safe
  direction.
- **It lowers the integral-authority requirement.** With the holding power
  supplied by feedforward, the integrator only covers the feedforward's model
  error (typically a small fraction of full duty), so `INTEGRAL_MAX` can be
  small.  The conditional-integration anti-windup (§9.5) automatically confines
  the integrator to the headroom the feedforward leaves.

#### Estimating `ambient_c`

`ambient_c` is the temperature the kettle sits at with no power.  The simplest
estimate is the cold-start temperature at the beginning of the step test, which
the script prints.  It need not be exact: any error in `ambient_c` (or `K`) just
becomes a small bias the integrator removes.  Feedforward is only active in auto
mode, and an RTD fault still forces manual control, so it never drives the
relays on bad data.

---

## 10. Data logging

`frontend/csv_logger.{h,cpp}` opens one timestamped file per run on startup:

```
~/bibby/logs/YYYY/MM/DD/HH-MM-SS.csv
```

Directories are auto-created. One row is written per **fresh** sensor sample,
right after the PID runs, and each row is `fflush`ed so a crash mid-brew keeps
the data on disk. Columns:

```
wall_time, t_monotonic_s, temp_raw_c, temp_filt_c, setpoint_c,
duty1, duty2, pid_output, pid_ff, pid_error, pid_p, pid_i, pid_d,
pid_integral, pid_deriv, manual, grain_in, rtd_fault, watchdog
```

- `pid_ff` is the holding-feedforward duty (§9.6); `pid_output` is the total
  commanded duty `clamp(pid_ff + pid_p + pid_i + pid_d)`.

- `wall_time` is ISO-8601 local time with milliseconds; `t_monotonic_s` is the
  SDL monotonic clock. Recover the sample interval `dt` from either.
- `grain_in` marks when grain was added; useful for aligning brew events to the
  temperature trace.

---

## 11. Repository layout

```
bibby/
├── CLAUDE.md                  project notes / coding conventions
├── CMakeLists.txt             top-level build (adds the two subprojects)
├── README.md                  this file
├── bibby.ini                  user configuration (mains, elements, PID, RTD cal)
├── docs/
│   └── MAX31865.pdf           sensor datasheet
├── heater-controller/         C — relay control process
│   └── main.c
├── frontend/                  C++ — UI / sensor / PID / logging process
│   ├── main.cpp
│   ├── display.{h,cpp}        rotated-FBO display + touch remap
│   ├── temp_filter.{h,cpp}    second-order moving average
│   ├── pid.{h,cpp}            PID controller
│   ├── csv_logger.{h,cpp}     per-run CSV logger
│   ├── sensors/
│   │   └── max31865.{h,cpp}   MAX31865 RTD driver
│   └── gui/
│       ├── kettle.{h,cpp}     kettle illustration
│       └── temp_history.{h,cpp} scrolling temperature graph
├── shared/
│   ├── shm_types.h            HeaterShm shared-memory layout
│   ├── config.{h,c}           bibby.ini loader (both processes)
│   └── single_instance.h      flock single-instance guard
├── tools/
│   └── identify_plant.py      FOPDT identification + PID tuning script (§9.4–9.5)
├── pi_hat/bibby_pi_hat/       KiCad schematic + PCB + gerbers
└── third_party/imgui/         vendored Dear ImGui (submodule)
```

---

## 12. License

bibby uses three licenses, one per artifact type:

| Artifact | License | File |
|---|---|---|
| Software (`frontend/`, `heater-controller/`, `shared/`) | Apache-2.0 | [`LICENSE`](LICENSE) |
| Hardware (`pi_hat/`) | CERN-OHL-P-2.0 | [`pi_hat/LICENSE`](pi_hat/LICENSE) |
| Documentation (`README.md`, `docs/`) | CC-BY-4.0 | [creativecommons.org/licenses/by/4.0](https://creativecommons.org/licenses/by/4.0/) |

**Apache-2.0** (software): permissive, includes an explicit patent grant so
techniques in the control code cannot be used against you or downstream users.
No per-file headers are required; the top-level `LICENSE` and `NOTICE` files
satisfy the license requirements.

**CERN-OHL-P-2.0** (hardware): the permissive variant of the CERN Open
Hardware Licence — the hardware analog of Apache/MIT. Covers the KiCad
schematics, PCB layout, and gerbers in `pi_hat/`.

**CC-BY-4.0** (documentation): attribution-only. You may reproduce, adapt,
and redistribute the documentation for any purpose as long as you credit the
original author.

Third-party attributions are in [`NOTICE`](NOTICE).

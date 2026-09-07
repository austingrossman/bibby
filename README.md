# bibby — Brew In A Bag Temperature Ramp Controller

`bibby` is a temperature and power controller for electric brew-in-a-bag (BIAB)
brewing, running on a Raspberry Pi 5. It reads kettle temperature from a PT100
RTD, drives two mains heating elements through solid-state relays (SSRs), and
presents a touch UI for setting and tracking the brew temperature. Power is
delivered as a duty cycle synchronized to the AC mains zero crossing, and the
whole system is built so that any fault, crash, or hang leaves the heaters
**off**.

This README is meant to let someone unfamiliar with the project understand it,
reproduce the hardware, build the software, and calibrate it for their own
kettle. It moves from the big picture down to the details: architecture, then
hardware, then software, then build/bring-up, then calibration.

This project is for fun. I am doing it to improve my home brewing set up. I hope
that someone sees inspiration in this project and does something similar. I am
doing this with a couple specific goals in mind:
- Learn to use AI for coding purposes.
- I recently purchased a Mac mini. I will only use a Mac for this project and it
  forces me to learn how to use it.
- I've recently acquired a 3D printer. This project has one 3D printed part, so
  I should learn how to use it.

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

The controller is a **single process** with three threads sharing one in-memory
state block of C11 atomics:

```
  +----------------------------------------------------------------+
  |                    bibby (one process)                          |
  |                                                                |
  |  main/UI thread          sampler thread         SSR thread     |
  |  ---------------         ------------------     -------------  |
  |  LVGL touch UI on        60 Hz, DRDY-paced:     SCHED_FIFO,    |
  |  /dev/fb0 (rotated)      - MAX31865 read        ZC-paced:      |
  |  - kettle readout        - temp filter          - zero-cross   |
  |  - charts, slider        - PID (watts)            capture      |
  |  - fault band            - power split          - sigma-delta  |
  |          |               - CSV logging            SSR firing   |
  |          |               - m*c estimator        - watchdogs    |
  |          v                     |                     |         |
  |  +------------------ BibbyState (C11 atomics) -------+         |
  |            duties, temps, modes, faults, history               |
  +----------------------------------------------------------------+
        | SPI0 + DRDY (GPIO16)      | GPIO20 ZC     | GPIO21/26
        v                           |               v
  +-------------+            +-------------+   +-----------+
  |  MAX31865   |            | zero-cross  |   |   SSR1    |
  |  + PT100    |            | detector    |   |   SSR2    |
  +-------------+            +-------------+   +-----------+
        ^                          ^                |
        | RTD                      | AC mains       | switched mains
     kettle probe                  line sense       to heating elements
```

- The **SSR thread** owns the relays. It runs at real-time priority
  (`SCHED_FIFO`) so a GUI stall can never delay the zero-cross response or the
  watchdogs.
- The **sampler thread** owns the sensor and the control law. It publishes
  per-element duty commands into the shared state; it never touches the relays.
- The **UI thread** is a pure view/controller: it renders the state and writes
  user intent (setpoint, mode, manual watts). Nothing in it is on the control
  or safety path.


### Why a duty cycle synchronized to zero crossing?

The heating elements are resistive loads switched by zero-cross SSRs. Switching
only at the AC zero crossing eliminates the inrush/EMI of phase-angle control.
Power is modulated by choosing *which* mains half-cycles to pass — a duty cycle
in `[0, 1]`. A sigma-delta modulator (§2.3) spreads the fired cycles out evenly
so the average power tracks the command with minimal low-frequency flicker.

### Why watts, not duty?

The control law works in **watts**: the PID outputs a total power demand, and a
splitter turns watts into per-element duties using the configured element
ratings. This makes the tuned gains independent of which elements are fitted —
swap elements, update `bibby.ini`, and the loop still commands the same power
per degree of error. It also enables the equal-flux split and anti-scorch cap
(§2.4), which need to know real watts per cm².

---

## 2. Software architecture

### 2.1 Shared state — `src/state.{h,c}`

`BibbyState` is a single struct of C11 atomics shared by the three threads —
no locks on the hot paths. Key fields:

| Field | Writer | Purpose |
|---|---|---|
| `duty1`, `duty2` | sampler | per-element duty commands in `[0,1]` |
| `p_demand_w`, `p_delivered_w` | sampler | commanded and measured average power |
| `temp_raw_c`, `temp_filt_c`, `temp_valid` | sampler | latest sensor sample and validity |
| `setpoint_c`, `manual_mode`, `manual_power_w`, `grain_in`, `simulate_zc` | UI | user intent |
| `output1`, `output2`, `zc_count`, `watchdog_alarm` | SSR thread | fired state, ZC counter, ZC watchdog |
| `control_heartbeat` | sampler | bumped every loop pass; the SSR thread's staleness watchdog watches it |
| `rtd_fault`, `rtd_unresponsive`, `fault_forced_manual` | sampler | sensor-fault state |
| `mc_est_j_per_c`, `adaptive_scale` | sampler | thermal-mass estimate and gain scale |
| `running` | main | global shutdown flag |

A mutex-guarded ring buffer of `HistPoint`s (one per 0.5 s) feeds the UI
charts; it is the only locked structure, and only the UI and the sampler's
0.5 s slow tick touch it.

### 2.2 Sampler thread — `src/sampler_thread.c`

Paced by the MAX31865 DRDY edge (one conversion per mains cycle, so ~60 Hz):

1. **Wait** for a DRDY falling edge (500 ms timeout so a dead sensor cannot
   stall the loop — the read also checks the DRDY level directly, so a missed
   edge still yields the conversion).
2. **Read** the sensor (§2.5); fresh samples go through the temperature filter
   (§2.6) and into the shared state.
3. **Mode interlock:** any RTD fault, unresponsive sensor, or invalid
   temperature forces **manual mode at zero watts** — automatic control can
   never command power on bad data.
4. **Control law** (§2.4): manual watts pass straight through; in auto, the
   watts-based PID runs once per fresh sample.
5. **Power split** (§2.4) turns the demand into `duty1`/`duty2`.
6. **Slow tick (0.5 s):** compute delivered power from the SSR thread's fired
   counters, update the m·c estimator (§2.7), push a chart history point.
7. **CSV logging** (§10): one row per fresh sample (or per 0.5 s during an
   outage, so faults stay on disk).

### 2.3 SSR thread — `src/ssr_thread.c`, `src/sigma_delta.h`

Runs at `SCHED_FIFO` (real-time) priority, pinned to the AC zero crossing:

1. **Zero-cross capture.** Wait for a rising edge on GPIO20 (libgpiod v2 edge
   events) with a mains-derived timeout: half-period + 0.7 ms guard — 9.0 ms at
   60 Hz, 10.7 ms at 50 Hz.
2. **Sigma-delta modulation.** Each zero crossing, add each element's duty
   fraction to a per-channel accumulator; fire that SSR for the half-cycle when
   the accumulator reaches 1.0 and subtract 1.0. The accumulator is capped at
   2.0 to bound catch-up after forced-off stretches. This is a first-order
   delta-sigma DAC clocked by the mains: the average fired fraction equals the
   commanded duty with the switching energy pushed to high frequency.
3. **Zero-cross watchdog.** No edge within the timeout → both SSRs forced off,
   `watchdog_alarm` set. A lost mains-sense signal must not leave a relay
   latched on. `simulate_zc` (the UI's "ZC Sim" toggle) substitutes the timeout
   tick for the edge so the modulator can be exercised with no mains connected.
4. **Control-staleness watchdog.** The sampler bumps `control_heartbeat` every
   pass; if it stops advancing for ~2 s worth of zero crossings
   (`2 × mains_Hz`), duties are forced to zero — a wedged control loop must not
   leave the heaters running. (`BIBBY_TEST_WEDGE=<sec>` wedges the sampler once,
   5 s in, to demonstrate exactly this on the bench.)
5. **Fired counters.** Per-element counts of fired half-cycles let the sampler
   compute true delivered watts over each slow tick.

### 2.4 Control law — `src/control.c`, `src/power_split.c`

**PID (watts).** `p = kp·e`, integrator accumulates raw per-sample error
(`ki` is W/°C per sample), derivative is a raw per-sample delta. Output is
`clamp(ff + p + i + d, 0, max_power)` in watts. Anti-windup is **conditional
integration**: the integrator holds whenever the non-integral output is already
against a rail and the error would push further into it; a backstop clamps the
integral term to full output authority (`max_power / ki`). No hand-tuned clamp
constant exists — the bounds all derive from the configured elements.

**Holding feedforward.** `ff = (setpoint − ambient_c) / process_gain_c` watts,
clamped to `[0, max_power]` — the steady-state power needed to hold the
setpoint, supplied directly so the integrator only trims model error (§9.6).
Zero gain disables it.

**Grain-in gain set.** The "Grain In" toggle swaps in an alternate gain set
(`grain.kp/ki/kd`) and optionally caps power (`grain.max_power_w`) — the plant
changes when grain is added, and scorch risk rises. Zero grain gains mean
"same as main gains disabled" (auto commands no power in grain mode until they
are set).

**Adaptive gain scaling** (config-gated, off by default). The m·c estimate
(§2.7) divided by `adaptive.mc_ref_j_per_c` scales `kp`/`kd` within
`[scale_min, scale_max]` — a half-size batch gets half the gain without
retuning.

**Equal-flux power split.** Demand is split so both elements run at the same
surface power density (W/cm²): proportional to area, with saturation overflow
pushed to the other element and everything clamped to the element ratings.
With different elements (e.g. 5000 W/820 cm² + 5500 W/473 cm²) the duties are
deliberately unequal — same flux, different power. `power.max_flux_w_cm2`
optionally caps total demand at `flux × total_area` (anti-scorch; 0 disables).

### 2.5 MAX31865 RTD sensor — `src/max31865.c`

- PT100, **3-wire**, 400 Ω nominal reference resistor, on `/dev/spidev0.0` at
  1 MHz, `SPI_MODE_1`.
- **Init sequence:** set 3-wire + clear faults + mains filter notch → enable
  VBIAS (10 ms settle) → enable auto-conversion (20 ms) → discard the first 10
  conversions using DRDY polling (they settle after VBIAS/auto enable).
- **Mains filter:** config-register bit 0 selects the noise-rejection notch
  (set = 50 Hz, clear = 60 Hz). It is set from `mains.frequency_hz` (§7) in the
  first config word — before auto-conversion, as the datasheet requires.
- **`max31865_read()` is non-blocking:** it checks the DRDY level (GPIO16,
  active-low); not ready → previous value with `fresh=false`. Otherwise it
  burst-reads RTD + fault registers in one SPI transaction, converts via
  Callendar–Van Dusen, and applies the per-unit `Rref`/gain/offset calibration
  (§9.1–9.2).
- **Fault handling:** the RTD-LSB fault bit and Fault Status register are
  checked every read. A faulted conversion is rejected (the stale value is held
  so the filter is not poisoned) and the latch cleared so the status reflects
  the next conversion. `max31865_fault_text()` decodes the register for the UI
  fault band.

The MAX31865 datasheet is included at [`docs/MAX31865.pdf`](docs/MAX31865.pdf).

### 2.6 Temperature filter — `src/temp_filter.c`

Cascaded boxcar (moving-average) stages — order and window from `bibby.ini`
(default 2 × 40 samples ≈ a second-order CIC with triangular weighting, ~0.65 s
group delay at 60 Hz). The first sample primes every stage so the output starts
at the true temperature instead of ramping from zero.

### 2.7 Thermal-mass estimator — `src/mc_estimator.c`

During any stretch of roughly constant delivered power where the temperature
rises ≥ 1 °C, the kettle behaves as an integrator: `dT/dt = P/(m·c)`. A
least-squares slope over the stretch gives `m·c = P/slope` (water ≈ 4.186 kJ/°C
per litre — the estimate is effectively the batch size). It is always computed
and logged; feeding it back into the gains is the separate, config-gated
adaptive scaling of §2.4.

### 2.8 UI — `src/ui/`

LVGL 9.3 rendering directly to the framebuffer:

- **Display:** `lv_linux_fbdev` on `/dev/fb0`; the physical panel is portrait,
  mounted sideways, so the UI renders 1280×720 and LVGL software-rotates 90°
  into the 720×1280 framebuffer (`ui.rotation`). Touch comes from the first
  evdev device advertising absolute multitouch (`ui.touch_device = auto`).
- **Layout:** toggles (ZC Sim, Grain In, Manual Control) and ±10/±1/±0.1
  setpoint steppers on top; temperature chart (setpoint, filtered, raw, grain
  and mode-change markers) and power/PID-term chart below; vertical power
  slider (manual watts in manual, live demand readout in auto); kettle graphic
  whose element glow follows the commanded duties; fault band (decoded RTD
  faults, watchdog, forced-manual notice); status line (m·c estimate and
  adaptive scale).
- LVGL is pinned to **v9.3.0**: v9.4 breaks the fbdev software-rotation path
  and hangs in `lv_deinit` on shutdown.
- The kernel console shares `/dev/fb0` and will draw its blinking cursor over
  the UI; the systemd unit unbinds it (§5.3).

`bibby --ui-test` shows a bring-up screen instead: border, corner labels, and a
crosshair that follows the finger — verifies rotation and touch mapping before
trusting the real UI.

---

## 3. Safety design

This system switches mains voltage into multi-kilowatt heating elements, so the
**default state is always off**. The design requirements and how they are met:

| Requirement | Mechanism |
|---|---|
| Heaters off at startup | SSR GPIO lines are requested with output value low before any thread runs |
| Heaters off on exit/crash | the kernel releases the GPIO lines when the process dies — for any reason — and the SoC pull-downs hold the SSR inputs low; SIGINT/SIGTERM additionally shut down in order |
| Only one instance | `flock` single-instance guard (`src/single_instance.h`); the kernel drops the lock on any exit |
| Heaters off if mains-sense (ZC) is lost | mains-derived zero-cross watchdog (~9 ms at 60 Hz) forces SSRs off, raises `watchdog_alarm` |
| Heaters off if the control loop wedges | control-staleness watchdog: sampler heartbeat stalled for ~2 s of ZCs → duties forced to zero |
| GUI stalls cannot delay safety logic | the SSR thread runs `SCHED_FIFO`; the UI is not on the control path |
| No power commanded on unreliable temperature | RTD fault / unresponsive sensor forces manual mode at zero watts; auto is unreachable until the sensor is healthy |
| Power commands always bounded | demand clamped to the configured element ratings (and the optional flux cap) before splitting; duties clamped to `[0,1]` |
| Controller returns after a crash | `Restart=always` in the systemd unit (§5.3); SSRs are safe during the gap per the kernel-release mechanism above |

---

## 4. Hardware

### 4.1 Major components

| Item | Notes |
|---|---|
| Raspberry Pi 5 | GPIO chip is `/dev/gpiochip4` on Pi 5 |
| Touchscreen display | 720×1280 DSI panel (Goodix touch), mounted sideways; UI software-rotates 90° |
| PT100 RTD probe | 3-wire, immersed in the kettle |
| 2 × zero-cross SSR | One per heating element; switched by GPIO21 / GPIO26 |
| 2 × resistive heating element | Mains-powered, BIAB kettle (this build: 5000 W and 5500 W) |
| Custom Pi HAT PCB | Sensor front-end + zero-cross detector (§4.2) |

> **TODO (fill in your build):** exact display model, SSR part numbers and
> current rating, fusing/contactor, and mains connector. These are
> deployment-specific and not captured in the repo.

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
- `R3`–`R6` = 43 kΩ, `R2` = 20 kΩ, `R7` = 1 MΩ (! DNI !) — line current-limiting
  and pull/bias network for the optocoupler. The pulse train feeds GPIO20.
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
| 36 | GPIO16 | DRDY | In | MAX31865 DRDY (sampler thread) |
| 37 | GPIO26 | SSR2 | Out | SSR thread |
| 38 | GPIO20 | Zero Cross | In | SSR thread |
| 40 | GPIO21 | SSR1 | Out | SSR thread |

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

- The H11AA1 provides **galvanic isolation** between the mains line-sense and
  the Pi logic.
- The SSRs are driven by 3.3 V GPIO on their input side; their output side
  switches mains into the elements.
- The RTD is a low-voltage measurement isolated from the mains by the kettle
  and the sensor front-end.

> **WARNING:** mains wiring, fusing, grounding/earth bonding, and enclosure
> safety are the integrator's responsibility. Use appropriately rated SSRs and
> heat-sinking, a proper earth connection to the kettle, and a
> contactor/fuse/breaker sized to the elements.

### 4.5 Enclosure

> **TODO (fill in your build):** enclosure material and dimensions, panel cutout
> for the display, ventilation/heat-sinking for the SSRs, and how the Pi + HAT +
> SSRs are mounted. Add photos and mechanical drawings here.

---

## 5. Raspberry Pi bring-up and installation

Tested target: **Raspberry Pi 5**, 64-bit Raspberry Pi OS.

### 5.1 OS and interfaces

1. Flash Raspberry Pi OS (64-bit) and boot the Pi. Install a beefy SD card (or
   other storage) for logging. Find a way to remote-login to the Pi.
2. Enable SPI:
   ```
   sudo raspi-config   # Interface Options -> SPI -> Enable
   ```
   Confirm `/dev/spidev0.0` exists after reboot.
3. The DSI touch display is auto-detected (`display_auto_detect=1` in
   `/boot/firmware/config.txt`). The touch controller registers as a Goodix
   evdev device; bibby finds it automatically.
4. The Pi 5 exposes GPIO via `/dev/gpiochip4`; the code opens it directly. The
   user running bibby needs the `gpio`, `spi`, `video`, and `input` groups (or
   run via the systemd unit, §5.3).

### 5.2 Build dependencies

```
sudo apt update
sudo apt install -y build-essential cmake git libgpiod-dev
```

- **libgpiod v2** is required (the code uses the v2 edge-event API). Confirm
  with `gpiodetect` / `pkg-config --modversion libgpiod`.
- **LVGL** is fetched and built from source by CMake (FetchContent) — nothing
  to install. It renders straight to the framebuffer; no X, Wayland, or GL
  stack is needed.
- The analysis tools (§9.4) additionally want
  `pip3 install numpy scipy matplotlib pandas` on whatever machine runs them.

### 5.3 Autostart at boot (systemd)

[`deploy/bibby.service`](deploy/bibby.service) is the reference unit. Besides
starting bibby it handles three things the app cannot do for itself:

- **Unbinds the kernel console from the framebuffer** (`vtcon1`) before start —
  otherwise the console's blinking cursor (and any kernel message) draws over
  the UI — and rebinds it on stop.
- **`LimitMEMLOCK=64M`** so `mlockall(MCL_CURRENT)` succeeds (the default 8 MB
  is too small once LVGL is linked in).
- **`AmbientCapabilities=CAP_SYS_NICE`** so the SSR thread gets its `SCHED_FIFO`
  real-time priority without running the whole process as root.

Install:

```
sudo cp deploy/bibby.service /etc/systemd/system/
sudo systemctl daemon-reload
sudo systemctl enable --now bibby
```

Check status and logs with `systemctl status bibby` / `journalctl -u bibby -f`.
Adjust `User=` and the paths in the unit for your username/checkout. The unit
uses `Restart=always`: the controller comes back if it ever dies, and the SSRs
are safe during the gap (§3).

---

## 6. Building bibby

```
git clone <repo-url> bibby
cd bibby
cmake -S . -B build -DCMAKE_BUILD_TYPE=Release
cmake --build build -j
```

The first configure fetches LVGL (pinned to v9.3.0 — see §2.8) from GitHub.
Two binaries result:

- `build/bibby` — the controller (links `lvgl`, `gpiod`, `m`, `pthread`, `rt`)
- `build/tests` — unit tests for the pure modules (no hardware needed):

```
./build/tests
```

`build/` and `logs/` are gitignored. There is no install step yet; run from the
build tree (the systemd unit does the same). The config file is found relative
to the executable (§7), so a copied-out binary works if `bibby.ini` sits next
to it.

---

## 7. Configuration

Runtime parameters that depend on the install (mains region, heating elements,
control gains, calibration) live in **`bibby.ini`**, **not** in the source.
There is no reason to edit a constant in code and rebuild to adapt a new brew
setup.

**Lookup order:** `-c <path>` argument → `$BIBBY_CONFIG` environment variable →
`bibby.ini` next to the executable → `bibby.ini` in the current directory.
A missing file or unknown key falls back to a built-in safe default (60 Hz,
zero gains), so the system still comes up. Comments (`#` or `;`, inline
allowed) and blank lines are ignored.

All keys, with their defaults:

| Key | Default | Effect |
|---|---|---|
| `mains.frequency_hz` | 60 | ZC watchdog timeout (half-period + 0.7 ms), staleness cutoff (2×Hz ZCs ≈ 2 s), MAX31865 notch (50/60 Hz) |
| `element1.watts` / `element2.watts` | 2500 / 2500 | element ratings; their sum is the maximum power demand |
| `element1.area_cm2` / `element2.area_cm2` | 150 / 150 | wetted areas driving the equal-flux split |
| `power.max_flux_w_cm2` | 0 (off) | anti-scorch cap: total demand ≤ flux × total area |
| `pid.kp` / `ki` / `kd` | 0 | watts-based gains (§9.5); zero = auto commands no power |
| `grain.kp` / `ki` / `kd` | 0 | alternate gain set while "Grain In" is on |
| `grain.max_power_w` | 0 (off) | power cap while grain is in |
| `feedforward.process_gain_c` | 0 (off) | identified K [°C/W]; holding feedforward `(setpoint−ambient)/K` watts |
| `feedforward.ambient_c` | 20 | feedforward reference temperature |
| `sensor.ref_resistor_ohms` | 400 | MAX31865 Rref, ice-point trimmed (§9.1) |
| `sensor.temp_cal_gain` / `temp_cal_offset` | 1.0 / 0.0 | two-point span trim, `T_true = g·T + b` (§9.2) |
| `filter.order` / `filter.window` | 2 / 40 | boxcar cascade stages and window length (§2.6) |
| `logging.rate` | high | `high` = row per sample (60 Hz); `low` = row per `low_period_s` |
| `logging.low_period_s` | 2.0 | row period in low-rate mode |
| `ui.show_zc_sim` | true | show the bench-test ZC Sim toggle (hide for a production panel) |
| `ui.chart_window_min` | 5 | chart time window, minutes (1–30) |
| `ui.rotation` | 90 | UI rotation onto the panel: 0/90/180/270 |
| `ui.fb_device` / `ui.touch_device` | auto | `auto` = `/dev/fb0` / first multitouch evdev; or explicit paths |
| `adaptive.enable` | false | scale kp/kd by (m·c estimate / `mc_ref_j_per_c`) |
| `adaptive.mc_ref_j_per_c` | 0 | m·c the gains were tuned at |
| `adaptive.scale_min` / `scale_max` | 0.5 / 4.0 | clamp on the adaptive scale |

The shipped `bibby.ini` documents each value's derivation inline; §9 gives the
full procedures.

---

## 8. Running

```
./build/bibby                  # the normal touch UI
./build/bibby --ui-test        # display/touch bring-up screen (§2.8)
./build/bibby --headless       # no display: prints a status line every 2 s
./build/bibby --sim-zc         # start with simulated zero crossings (bench)
./build/bibby --manual-w 2000  # start in manual at 2000 W (step tests)
./build/bibby -c /path/to.ini  # explicit config
```

If the framebuffer cannot be opened the UI falls back to headless mode rather
than exiting — the controller and safety logic keep running.

### UI elements

- **Kettle card:** setpoint, measured °C and °F, animated elements whose glow
  tracks the commanded duty, per-element duty readout (`E1 x%  E2 y%` — these
  differ by design; see the equal-flux split, §2.4).
- **POWER slider:** commands watts in manual mode; shows the loop's live demand
  in auto (disabled for input). Total watts read out under it.
- **Setpoint steppers:** ±10 / ±1 / ±0.1 °C.
- **Toggles:** ZC Sim (simulate zero crossings — bench only), Grain In (gain
  set swap + logged event marker), Manual Control (manual watts vs PID auto).
- **Charts:** temperature (setpoint / filtered / raw + grain and mode-change
  markers) and power (demand W / delivered / ff / P / I / D), window set by
  `ui.chart_window_min`, y-autoscaled.
- **Fault band:** decoded RTD faults, watchdog alarm, and "Auto disabled →
  manual" when a fault forced the mode switch.
- **Status line:** m·c estimate (≈ batch size) and adaptive gain scale.

**Bench testing without mains:** toggle **ZC Sim** (or start with `--sim-zc`).
The SSR thread then clocks the sigma-delta from its timeout tick instead of the
missing zero-cross edge; the full UI, control law, and logging run with no AC
connected.

---

## 9. Calibration methodology

Three of the measurement constants are **per-unit** — they depend on the
specific RTD probe and the board's reference resistor, so they **must be
re-derived for any other board or probe**. All three live in the `[sensor]`
section of `bibby.ini`; **none require a code change or rebuild**. Their
identity defaults (`Rref` = 400 Ω, gain = 1.0, offset = 0.0) give a working but
uncalibrated sensor.

`tools/calibrate_sensor.py` does the §9.1/§9.2 arithmetic for you (standard
library only):

```
python3 tools/calibrate_sensor.py rref --ice-reading 1.4 --rref-current 400
python3 tools/calibrate_sensor.py span --p1 0.0 0.0 --p2 25.0 23.78
```

| `bibby.ini` key | Symbol | What it corrects |
|---|---|---|
| `sensor.ref_resistor_ohms` | `Rref` | one-point offset at the ice point (§9.1) |
| `sensor.temp_cal_gain` | `g` | span/slope error across temperature (§9.2) |
| `sensor.temp_cal_offset` | `b` | residual offset after the span fit (§9.2) |

The driver applies them in this order, so **calibrate in this order too**:

```
R_rtd  = (raw / 32768) * Rref            # ref_resistor_ohms  (§9.1)
T_cvd  = CVD(R_rtd)                      # Callendar–Van Dusen, fixed
T_true = g * T_cvd + b                   # gain, offset       (§9.2)
```

### 9.1 Reference resistor `Rref` (ice-point one-point trim)

`Rref` scales the entire resistance reading, so trimming it nulls a constant
offset. Trim it first, at a single known low temperature (the ice point is the
cheapest 0 °C reference available).

**Procedure**

1. Set `sensor.temp_cal_gain = 1.0`, `sensor.temp_cal_offset = 0.0` (so the
   span trim does not mask the reading), and `sensor.ref_resistor_ohms` to the
   board's nominal reference (400 Ω here — the value of `R1` on the HAT, §4.2).
2. Submerge the probe in a well-stirred ice-water bath and let it settle.
3. Read the measured temperature `T_ice` (the **raw** column in the CSV log,
   §10, before any span trim).
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
the best fit. Good choices: the ice point (0 °C) and the local **boiling
point** (altitude-corrected, below). A stirred ambient bath against a trusted
reference thermometer also works for the upper point.

**Boiling-point reference.** Water boils below 100 °C at altitude; approximate
the local boiling point as `T_boil ≈ 100 − 3.4·(h/1000m)` (°C, `h` =
elevation), or read it from a steam table for your barometric pressure, then
use that as the upper `T_ref`.

**Worked example (this unit):** ice point `(0.00, 0.00)` and a fan-mixed
ambient point `(25.00, 23.78)`:
`gain = (25.00 − 0.00)/(23.78 − 0.00) = 1.0513`,
`offset = 0.00 − 1.0513·0.00 = 0.0`. After a small refinement the shipped
values are `sensor.temp_cal_gain = 1.0528`, `sensor.temp_cal_offset = -0.032`.

> **PROVISIONAL:** both points above are ≤ 25 °C. Re-fit using a boiling-point
> upper reference before trusting brew-range temperatures — at 5230 ft the
> local boiling point is **94.7 °C**, and the §9.1-trimmed RTD should read
> ~90.0 °C there. Re-derive both keys for your own probe and altitude.

### 9.3 Process model

The model of the signal chain from power command to measured temperature has
three cascaded stages: the kettle thermal plant, the RTD probe lag, and the
software filter.

#### Signal chain

```
  P [W] ──►[ G_plant ]──► T_kettle ──►[ G_probe ]──►[ G_filt ]──► T_meas ──► PID
            K/(τs+1)                  1/(τ_pr s+1)   boxcar cascade

  plant heat loss: T_kettle relaxes toward ambient at rate k_loss·(T − T_amb)
```

#### Kettle thermal plant

Energy balance of a well-stirred (recirculated) kettle, linearised about an
operating point:

```
m · c_p · dT/dt = P − k_loss · (T − T_amb)

G_plant(s) = T(s)/P(s) = K / (τ · s + 1)

  K       = 1/k_loss        [°C/W]   steady-state gain
  τ       = m·c_p / k_loss  [s]      dominant time constant

  m       liquid mass, kg
  c_p     4186 J/(kg·°C) for water
  k_loss  total heat-loss coefficient, W/°C
          (conduction through walls + lid, radiation, evaporation)
```

For a 30 L batch in a reasonably insulated kettle τ is typically **20–40
minutes**. The initial ramp rate at power `P` is `P/(m·c_p)` — easy to check
directly against the step test, and the basis of the online m·c estimator
(§2.7).

#### RTD probe lag

The PT100 probe has its own thermal mass causing a first-order lag:

```
G_probe(s) = 1 / (τ_probe · s + 1),   τ_probe ≈ 5–30 s
```

#### Software filter

The default filter is two cascaded N = 40 boxcars at the sensor rate
`f_s` (≈ 60 Hz — one conversion per mains cycle). Each boxcar contributes
`(N−1)/(2·f_s) ≈ 0.33 s` of group delay; both together ≈ 0.65 s, absorbed into
the dead time below.

#### Combined FOPDT approximation

```
G_FOPDT(s) = K · e^(−L·s) / (τ · s + 1)

  K   ≈ 1/k_loss                [°C/W]
  τ   ≈ m·c_p / k_loss          [s]
  L   ≈ τ_probe + filter delay  [s]
```

The ratio `L/τ` characterises controllability: `< 0.3` is straightforward;
`> 1.0` is delay-dominated. A BIAB kettle sits well below 0.3 — the challenge
is the large time constant, not the dead time.

#### Gain conventions — continuous vs. INI

The PID accumulates raw error once per sample (`integral += e`, not `e·dt`)
and uses raw per-sample deltas for the derivative. The INI gains relate to
continuous-time gains by:

```
kp_ini = Kp_c            [W/°C, unchanged]
ki_ini = Ki_c · T_s      [T_s = sample period ≈ 1/60 s]
kd_ini = Kd_c / T_s
```

`identify_plant.py` measures `T_s` from the log and folds it in automatically.

### 9.4 System identification

`K`, `τ`, and `L` come from an **open-loop step test**: command constant watts
in manual mode and record the temperature response.
`tools/identify_plant.py` fits the model and prints ready-to-paste
`[pid]`/`[feedforward]` blocks.

#### Step-test procedure

1. **Fill** the kettle to the intended brew volume and start the recirculation
   pump. Good mixing is required for the lumped-thermal-mass model to hold.
2. **Stabilise.** Let the temperature settle a few minutes at 0 W.
3. **Apply the step.** Manual Control on; set the power slider to **30–50 %**
   of maximum (a clear signal, well clear of boiling for ≥ 15 min); leave it.
   (`--manual-w` on the command line does the same from startup.)
4. **Log.** The CSV logger runs continuously. Let it run until the temperature
   has risen **15–25 °C**, or ≥ **15 minutes** — whichever comes first. Longer
   is better: a test that approaches steady state pins down `K` and `τ`
   individually, not just their ratio.
5. **End.** Slider back to 0. Note the time to locate the step in the log.
6. **Copy the log** from `~/bibby/logs/YYYY/MM/DD/HH-MM-SS.csv`.

> If the liquid reaches boiling, stop — the model changes once evaporative
> cooling is significant.

#### Running the identification

```
pip3 install numpy scipy matplotlib pandas    # once

python3 tools/identify_plant.py logs/YYYY/MM/DD/HH-MM-SS.csv \
        --t-start 120 --t-end 1020 -o step_fit.png

# Specify the step power explicitly (otherwise the log median is used),
# and λ for a more conservative IMC tuning:
python3 tools/identify_plant.py logs/... --watts 4000 --lambda 60
```

`--t-start`/`--t-end` are seconds from the start of the log; trim the window
to the step. Old duty-format logs (pre-rebuild) are converted with
`--e1-watts`/`--e2-watts`.

The script prints the FOPDT fit, a physical cross-check — `k_loss`, `m·c`
(compare against the actual litres × 4.186 kJ/°C), and the °C/min ramp rate —
tuned gains per rule, and the INI blocks. The plot shows the measured
temperature, the fit, the power trace, and residuals; inspect the residuals to
confirm the model fits.

| Parameter | Physical meaning | Implication |
|---|---|---|
| `K` | 1 / heat-loss coefficient | Large K: heater dominates; steady state far above ambient |
| `τ` | Thermal time constant of the batch | Larger batch = larger τ; the loop can afford slower response |
| `L` | Probe lag + filter delay | Limits tuning aggression; keep the probe well-immersed |
| `L/τ` | Relative dead time | < 0.1 easy; > 0.5 requires careful detuning |
| `m·c` | Batch thermal mass | Sanity check vs. litres; reference for adaptive scaling |

### 9.5 PID tuning

Auto mode commands zero power until `pid.kp/ki/kd` are set. The script offers
three rules, all from the same fit:

| Rule | Characteristic | When to use |
|---|---|---|
| **IMC-PI** | Smooth, no overshoot; λ sets speed vs. robustness | First choice for BIAB; tune λ to taste |
| **ZN-PID** | Aggressive, ~25 % overshoot | Upper bound on aggressiveness only |
| **CC-PID** | Balanced at moderate `L/τ` | Sanity check against IMC |

For a mash, **IMC-PI** with `λ ≈ L` is a good start. Overshoot costs enzyme
activity — err toward larger λ. Keep `kd = 0`: at 60 Hz the raw per-sample
derivative mostly amplifies sensor noise.

The shipped `bibby.ini` carries IMC-PI starting gains identified from this
build's 2026-06-20 bench tests (~15 L, 5000+5500 W elements). **Validate on
your own setup before trusting a batch:**

1. Copy the printed `[pid]` (and `[feedforward]`, §9.6) blocks into
   `bibby.ini` and restart bibby.
2. Set a setpoint **5–10 °C above** current temperature, switch to **Auto**,
   and watch the charts.
3. A good response rises smoothly and settles with minimal overshoot. If it
   oscillates, increase λ and re-run the script.
4. Inspect the logged `pid_ff_w`/`pid_p_w`/`pid_i_w`/`pid_d_w` split: the
   feedforward should carry the holding power, the integral only trim, the
   derivative not chatter.

Anti-windup needs no configuration: conditional integration plus the
`max_power/ki` backstop (§2.4) bound the integrator automatically.

**Grain-in gains.** Adding grain changes the plant (more mass, worse mixing,
scorch risk at the bag). Re-run the step test with grain in (or a sacrificial
equivalent) to derive `grain.kp/ki/kd`, and consider `grain.max_power_w` to cap
flux at the bag.

**Adaptive scaling.** If batch size varies brew to brew, set
`adaptive.mc_ref_j_per_c` to the m·c the gains were tuned at and enable
`adaptive.enable`: kp/kd then scale with the measured batch size within
`[scale_min, scale_max]`.

### 9.6 Feedforward (optional, recommended)

A kettle spends most of a brew *holding* a temperature against heat loss. Left
to the PID alone, the integrator must wind up to supply that holding power —
slow to converge and a source of overshoot. The **holding feedforward**
supplies it directly:

```
ff_w    = clamp( (setpoint − ambient_c) / K , 0, max_power )   [watts]
demand  = clamp( ff_w + P + I + D , 0, max_power )
```

`K` (°C/W) and `ambient_c` live in `[feedforward]`; the same step test that
tunes the PID sizes the feedforward, and the script prints the block.
`process_gain_c = 0` disables it (the safe default).

- **It does not change loop stability.** `ff_w` depends on the *setpoint*, not
  the measurement, so it sits outside the feedback loop; the closed-loop
  dynamics are unchanged (superposition).
- **It does not cause overshoot** unless over-sized (K under-estimated). Keep K
  honest; a slight *under*-estimate of the holding power is the safe direction.
- **Beware short step tests.** A test that never approaches steady state
  cannot pin down `K` — only `m·c` (the ramp slope) is well-determined. The
  script warns in this case; do not enable feedforward from such a fit. This
  build ships with feedforward disabled for exactly that reason.
- `ambient_c` is the temperature the kettle sits at unpowered; the window-start
  temperature of a cold step test is a fine estimate. Errors just become a
  small bias the integrator removes.

Feedforward is only active in auto mode, and an RTD fault still forces manual,
so it never drives the relays on bad data.

---

## 10. Data logging

`src/csv_logger.c` opens one timestamped file per run:

```
~/bibby/logs/YYYY/MM/DD/HH-MM-SS.csv
```

Directories are auto-created. In the default high-rate mode one row is written
per **fresh** sensor sample (~60 Hz), right after the control law runs, and
each row is flushed so a crash mid-brew keeps the data on disk. During a sensor
outage a row is still written every 0.5 s so the fault interval is on disk.
`logging.rate = low` drops to one row per `logging.low_period_s` for long
unattended runs. Columns:

```
wall_time, t_monotonic_s, temp_raw_c, temp_filt_c, setpoint_c,
p_demand_w, p_delivered_w, duty1, duty2, flux1_w_cm2, flux2_w_cm2,
pid_ff_w, pid_p_w, pid_i_w, pid_d_w, pid_integral, pid_deriv, pid_error_c,
mc_est_j_per_c, manual, grain_in, rtd_fault, watchdog
```

- `p_demand_w` is the commanded power; `p_delivered_w` is measured from the
  SSR fired counters over each 0.5 s tick — they differ while the watchdog
  holds the outputs off, which is itself useful data.
- `flux1/2_w_cm2` are the per-element surface power densities from the split.
- `pid_*_w` is the feedforward / P / I / D breakdown in watts (§9.5);
  `pid_error_c` the residual error.
- `wall_time` is ISO-8601 local with milliseconds; `t_monotonic_s` is
  `CLOCK_MONOTONIC`. Recover `dt` from either.
- `grain_in` marks grain addition; `manual` the control mode; `rtd_fault` the
  raw MAX31865 fault byte; `watchdog` the ZC watchdog state.

`tools/plot_logs.py` is a small Tk browser for the log tree — pick a file,
tick columns, zoom. It reads whatever columns the header declares, so both old
and new logs open fine.

---

## 11. Repository layout

```
bibby/
├── CLAUDE.md                  project notes / coding conventions
├── CMakeLists.txt             build: bibby + tests, fetches LVGL
├── README.md                  this file
├── bibby.ini                  user configuration (mains, elements, gains, cal)
├── deploy/
│   └── bibby.service          systemd unit (console unbind, memlock, RT prio)
├── docs/
│   ├── MAX31865.pdf           sensor datasheet
│   └── REBUILD.md             the plan the single-process rebuild followed
├── src/
│   ├── main.c                 startup, GPIO requests, thread spawn, shutdown
│   ├── state.{h,c}            shared atomics + chart history ring
│   ├── config.{h,c}           bibby.ini loader
│   ├── control.{h,c}          watts PID + feedforward
│   ├── power_split.{h,c}      equal-flux watts→duty split + flux cap
│   ├── sigma_delta.h          per-ZC sigma-delta modulator
│   ├── ssr_thread.{h,c}       ZC capture, SSR firing, watchdogs (SCHED_FIFO)
│   ├── sampler_thread.{h,c}   sensor pacing, filter, control, logging
│   ├── max31865.{h,c}         MAX31865 RTD driver
│   ├── temp_filter.{h,c}      boxcar cascade filter
│   ├── mc_estimator.{h,c}     online thermal-mass estimate
│   ├── csv_logger.{h,c}       per-run CSV logger
│   ├── single_instance.h      flock single-instance guard
│   └── ui/
│       ├── lv_conf.h          LVGL configuration
│       ├── ui.{h,c}           LVGL init, fbdev + rotation, evdev touch, ui-test
│       ├── ui_screen.{h,c}    main screen: charts, slider, toggles, faults
│       ├── ui_kettle.{h,c}    kettle illustration with element glow
│       └── ui_theme.h         colors
├── tests/
│   └── test_units.c           unit tests for the pure modules
├── tools/
│   ├── identify_plant.py      FOPDT identification + PID tuning (§9.4–9.5)
│   ├── calibrate_sensor.py    §9.1/§9.2 calibration arithmetic
│   └── plot_logs.py           CSV log browser/plotter
└── pi_hat/bibby_pi_hat/       KiCad schematic + PCB + gerbers
```

---

## 12. License

bibby uses three licenses, one per artifact type:

| Artifact | License | File |
|---|---|---|
| Software (`src/`, `tests/`, `tools/`, `deploy/`) | Apache-2.0 | [`LICENSE`](LICENSE) |
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

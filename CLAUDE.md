# bibby - Brew In a Bag Temperature Ramp Controller

Embedded temperature/power controller running on Raspberry Pi 5. **One process,
three threads** — `bibby` (C11) does SSR firing, RTD sampling, the control law, the
touchscreen UI, and CSV logging. The earlier two-process split (frontend +
heater-controller over POSIX shared memory, SDL3/ImGui UI) is gone; the plan that
rebuild followed is in `docs/REBUILD.md`. README §2 has the architecture diagrams.

## Threads

### main — `src/main.c`
- Parses args (`-c <ini>`, `--headless`, `--ui-test`, `--sim-zc`, `--manual-w W`), takes a
  flock single-instance lock (`/tmp/bibby.lock`, `src/single_instance.h`), loads the config.
- Requests all GPIO lines up front via libgpiod v2, **before any thread runs**: SSR outputs
  are granted `INACTIVE` (elements off at startup), ZC as rising-edge input, DRDY as
  falling-edge input. The kernel releases the lines on any exit, and the SoC pull-downs
  hold the SSR inputs low — elements off on crash.
- Spawns the SSR and sampler threads with 512 KB stacks, then `mlockall(MCL_CURRENT)`
  (deliberately not `MCL_FUTURE`: locking later UI allocations against the memlock budget
  makes them fail outright).
- Then runs the LVGL UI on this thread (`ui_run_mode`). If the framebuffer can't be opened
  the UI falls back to `headless_loop()` — control and safety keep running either way.
- GPIO chip `/dev/gpiochip4` (Pi 5). Sensor init failure is non-fatal: manual mode only.

### ssr thread — `src/ssr_thread.c`
- `SCHED_FIFO` prio 20 (needs `CAP_SYS_NICE`; warns and continues without it), so UI stalls
  can never delay zero-cross response or the watchdogs.
- Blocks on the ZC rising edge (GPIO 20) with `config_zc_timeout_ns()` as the timeout.
- Sigma-delta duty modulation per ZC (`src/sigma_delta.h`): accumulate duty, fire when
  acc ≥ 1.0, accumulator capped at 2.0.
- **ZC watchdog:** timeout with no edge → both SSRs off, `watchdog_alarm` set. With
  `simulate_zc` on, that same timeout *is* the fake crossing (bench testing without mains).
- **Control-staleness watchdog:** the sampler bumps `control_heartbeat` every pass; if it
  freezes for `config_staleness_zc()` crossings (~2 s), both duties are forced to 0.
- Counts `zc_count`, `fired1`, `fired2` (the sampler turns fired half-cycles into measured
  delivered watts). Drives the SSRs low on the way out and clears `running`.

### sampler thread — `src/sampler_thread.c`
Everything paced by the RTD conversion rate; one pass = heartbeat + read + control + log.
- Waits on the DRDY falling edge (500 ms timeout), then reads regardless of the wait result
  — `max31865_read()` checks the DRDY level itself, so a missed edge can't stall the loop.
- Fresh sample → cascaded boxcar filter (`src/temp_filter.c`) → `temp_raw_c`/`temp_filt_c`.
- **Mode interlock:** any RTD fault bit, an unresponsive sensor (>1 s with no conversion),
  or no valid sample yet forces manual at 0 W and sets `fault_forced_manual`.
- Grain In swaps the gain set (`[grain]`) and optionally lowers peak power.
- Manual publishes duty every pass (a dead sensor must not freeze the slider); auto runs the
  control law once per fresh sample.
- Slow tick (0.5 s): measured delivered power from the fired counters, m·c estimate, chart
  history push (~2 Hz into the `HISTORY_CAP` ring).
- Logging: one CSV row per fresh sample; during a fault, one row per slow tick so the outage
  and its fault bits stay on disk.
- Bench hook: `BIBBY_TEST_WEDGE=<sec>` wedges this thread once, 5 s in, to exercise the
  staleness watchdog.

### UI — `src/ui/`
- LVGL 9.3.0 (pinned; 9.4 breaks rotation/shutdown), fbdev display + evdev touch, rotated in
  software by `ui.rotation` for the sideways-mounted panel. Touch device auto-probed (first
  evdev node with `ABS_MT_POSITION_X`) unless `ui.touch_device` is set.
- `src/ui/lv_conf.h` takes LVGL off its built-in fixed pool (`LV_USE_STDLIB_MALLOC =
  LV_STDLIB_CLIB`): the 64 KB default left ~22 KB of headroom on this screen and a big redraw
  asserted out of memory in `lv_draw_add_task`. `LV_ASSERT_HANDLER` is `abort()`, not the
  default `while(1);` — an assert must not wedge the UI thread at 100 % CPU with the elements
  live; dying drops the GPIO lines and lets `bibby.service` restart the process.
- `ui.c` = LVGL/backend init + the `--ui-test` bring-up screen (corner markers + crosshair,
  for checking rotation and touch mapping). `ui_screen.c` = the controller screen,
  `ui_kettle.c` = the kettle cutaway with element glow, `ui_theme.h` = the palette.
- Screen: kettle card (setpoint, °C/°F, per-element duty), POWER slider (commands watts in
  manual, read-only live demand in auto), ±10/±1/±0.1 setpoint steppers, ZC Sim / Grain In /
  Manual Control toggles, temperature chart, power chart (demand/delivered/ff/P/I/D), fault
  band, m·c + adaptive-scale status line. Refresh timer at 250 ms, charts every other tick.
- Each chart carries a row of legend chips; tapping one hides that trace and drops it from
  that chart's autoscale (`vis[]` / `trace[]` in `ui_screen.c`). All traces start visible.
- **The UI owns no control or safety logic.** It reads the state snapshot and writes only
  `setpoint_c`, `manual_mode`, `manual_power_w`, `grain_in`, `simulate_zc`.

## Control chain — the units matter
`°C error → watts → duty → fired half-cycles`. Duty is an *actuator* command, not a control
quantity; it appears for the first time in the power split.

- `src/control.{h,c}` — PID in watts (`kp` in W/°C; `ki`/`kd` are per-sample discrete gains
  folded with the sample period by `tools/identify_plant.py`). Holding feedforward
  `(setpoint − ambient)/K` watts is added ahead of the feedback, so the PID only trims model
  error. Conditional-integration anti-windup (hold the integrator when the pre-step output is
  already railed and the error pushes further in), plus an integral backstop of
  `out_max_w / ki`. `PidTerms` carries the per-term breakdown for logging/charts.
- `src/power_split.{h,c}` — splits the watts demand across the two elements at **equal
  surface flux** (anti-scorch), pushing overflow to the other element when one saturates.
  `power.max_flux_w_cm2 > 0` caps total demand at `flux × (A1 + A2)`. E1% and E2% differ by
  design.
- `src/mc_estimator.{h,c}` — online thermal mass: over a stretch of roughly constant
  delivered power with ≥1 °C rise, `m·c = P / (dT/dt)` in J/°C (≈ batch size). Always
  computed and logged; only `[adaptive]` decides whether it scales `kp`/`kd`.
- `src/temp_filter.{h,c}` — cascaded boxcar, `filter.order` stages × `filter.window` samples.
  Group delay `order*(window−1)/2` samples feeds the dead-time term used for tuning.

## Shared state — `src/state.h`
No shared memory and no IPC any more: one `BibbyState` of C11 atomics, since each scalar is
independent and readers tolerate one-sample skew. The only lock is `hist_mutex` around the
chart history ring.

| Field | Writer | Reader | Purpose |
|---|---|---|---|
| `duty1`, `duty2` | sampler | ssr | per-element duty command [0,1] |
| `simulate_zc` | ui | ssr | treat ZC timeouts as crossings (bench) |
| `control_heartbeat` | sampler | ssr | staleness watchdog |
| `output1`, `output2` | ssr | ui | SSR fired state this half-cycle |
| `watchdog_alarm` | ssr | sampler, ui | no ZC within the mains-derived timeout |
| `zc_count`, `fired1`, `fired2` | ssr | sampler | delivered-power measurement |
| `temp_raw_c`, `temp_filt_c`, `temp_valid` | sampler | ui | temperature |
| `rtd_fault`, `rtd_unresponsive` | sampler | ui | MAX31865 fault bits / silent DRDY |
| `p_demand_w`, `p_delivered_w`, `mc_est_j_per_c`, `adaptive_scale` | sampler | ui | loop telemetry |
| `fault_forced_manual` | sampler | ui | auto was blocked/kicked by a sensor fault |
| `setpoint_c`, `manual_mode`, `manual_power_w`, `grain_in` | ui | sampler | operator commands |
| `running` | any | all | clear to shut the process down |

## CSV Logging — `src/csv_logger.{h,c}`
- One timestamped file per run: `~/bibby/logs/YYYY/MM/DD/HH-MM-SS.csv` (dirs auto-created).
- Two rates, same schema, chosen by `[logging]`: `high` = every call (one row per fresh
  sample, ~50/60 Hz, for tuning); `low` = decimated to one row per `low_period_s`.
- Every row is `fflush`ed so a crash mid-brew keeps the data on disk.
- Columns: `wall_time, t_monotonic_s, temp_raw_c, temp_filt_c, setpoint_c, p_demand_w,
  p_delivered_w, duty1, duty2, flux1_w_cm2, flux2_w_cm2, pid_ff_w, pid_p_w, pid_i_w,
  pid_d_w, pid_integral, pid_deriv, pid_error_c, mc_est_j_per_c, manual, grain_in,
  rtd_fault, watchdog`. `wall_time` is ISO-8601 local with ms; there is no `dt` column —
  recover time from `wall_time` or `t_monotonic_s`.
- `tools/plot_logs.py` browses/plots the logs; `tools/identify_plant.py` drives the lumped
  kettle model with the logged `p_delivered_w` and fits m·c, k_loss, L by least squares
  (heat-then-cool test, whole log, `--ambient` required), then prints PID/feedforward blocks.

## MAX31865 Sensor — `src/max31865.{h,c}`
- PT100, 3-wire, continuous conversion → a fresh sample every mains-notch period (~50/60 Hz).
- SPI `/dev/spidev0.0`, 1 MHz, `SPI_MODE_1`.
- Init order (datasheet): 3-wire + notch + fault-clear → VBIAS on, 10 ms settle →
  auto-conversion → discard the first 10 conversions. The notch bit (`CONFIG_50HZ_FILTER`,
  config reg bit 0; set = 50 Hz, clear = 60 Hz) comes from `mains.frequency_hz` and **must**
  be chosen before auto-conversion is enabled.
- `max31865_read()` is non-blocking: if DRDY is not asserted or the conversion is flagged
  faulty it returns the previous value with `fresh = false`, so bad data never reaches the
  filter. Otherwise Callendar-Van Dusen → `T_true = cal_gain*T + cal_offset`.
- Every read refreshes the latched Fault Status register; `max31865_fault_text()` decodes the
  bits for the UI fault band.
- DRDY: GPIO 16 (pin 36), active low, falling edge; the `gpiod_line_request*` is passed into
  `max31865_init()` and stored.

## Pin assignments (40-pin header)
| Pin | GPIO | Signal | Direction | Used by |
|---|---|---|---|---|
| 1 | — | 3.3V | Power | MAX31865 VIN |
| 6 | — | GND | Power | MAX31865 GND |
| 19 | GPIO10 | SPI0 MOSI | Out | MAX31865 SDI |
| 21 | GPIO9 | SPI0 MISO | In | MAX31865 SDO |
| 23 | GPIO11 | SPI0 SCLK | Out | MAX31865 CLK |
| 24 | GPIO8 | SPI0 CS | Out | MAX31865 CS |
| 36 | GPIO16 | DRDY | In | MAX31865 DRDY (sampler thread) |
| 37 | GPIO26 | SSR2 | Out | ssr thread |
| 38 | GPIO20 | Zero Cross | In | ssr thread |
| 40 | GPIO21 | SSR1 | Out | ssr thread |

## Configuration — `src/config.{h,c}` + `bibby.ini`
- `bibby.ini` (repo root) holds everything another person/setup would change. Loaded once at
  startup by `config_load()`; unknown keys and a missing file leave the built-in defaults
  standing (60 Hz, zero gains → auto commands no power until tuned).
- Lookup order: `-c <path>` → `$BIBBY_CONFIG` → `bibby.ini` beside the executable
  (`/proc/self/exe`, so an installed binary finds its own config) → `./bibby.ini`.
  `config_resolved_path()` reports which one was used; startup logs it.
- INI format: `[section]` headers, `key = value`, `#`/`;` comments (inline too). Sections:
  `[mains] [element1] [element2] [power] [pid] [grain] [feedforward] [sensor] [filter]
  [logging] [ui] [adaptive]`. README §7 has the full key table with defaults.
- Derived helpers: `config_zc_timeout_ns()` = half-period + 0.7 ms guard (60 Hz → 9.0 ms,
  50 Hz → 10.7 ms); `config_staleness_zc()` = `2*hz` (~2 s); `config_total_watts()` = P1+P2,
  the control-law output clamp before the flux cap.
- `sensor.*` is the per-unit RTD calibration (ice-point Rref trim + two-point span fit);
  identity defaults (400 Ω / 1.0 / 0.0) = uncalibrated. Derivations are inline in `bibby.ini`
  and in README §9.

## Build and tests
CMake, C11, `-Wall -Wextra`. LVGL v9.3.0 via FetchContent (`src/ui/lv_conf.h` is the config).
Two targets: `bibby` (links `lvgl`, `gpiod`, `m`, `pthread`, `rt`) and `tests`
(`tests/test_units.c` + the pure modules — power split, sigma-delta, control, temp filter,
m·c estimator, config; no hardware, no LVGL).
`build/` and `logs/` are gitignored. `bibby.ini` and `deploy/bibby.service` are checked in.

## Development workflow (Austin's setup — environment-specific, not a build requirement)
The repo lives **on the Pi** at `/home/ramp/bibby`. The Mac mini mounts it at
`/Users/austin/bibby` over sshfs, so both machines see the same working tree and edits
propagate instantly in both directions. The mount is automatic: a LaunchAgent
(`com.austin.bibby-mount`, script `~/.local/bin/bibby-mount`) remounts it within ~60s
whenever the Pi is reachable, and no-ops silently when the Pi is powered off.

- VS Code and Claude Code run **on the Mac**. A shell opened there is a *Mac* shell —
  editing files under `/Users/austin/bibby` is editing the Pi's files, but running a
  command there runs it on the Mac.
- **Anything that touches the hardware or the toolchain must go over ssh**: `cmake`
  builds, launching `bibby`, `gpioinfo`, SPI, framebuffer screenshots. Use the `ramp`
  host alias (passwordless key, passwordless sudo):
  `ssh ramp 'cd ~/bibby && cmake --build build --target bibby -j4'`
- Build directory is `~/bibby/build` **on the Pi**; the binary is `build/bibby`. The
  hardware-free unit tests build there as well:
  `ssh ramp 'cd ~/bibby && cmake --build build --target tests -j4 && ./build/tests'`.
  Never build from the Mac — the toolchain, libgpiod, and LVGL's fbdev/evdev backends
  are aarch64/Pi-only.
- `bibby` normally runs as the `bibby.service` systemd unit (`deploy/bibby.service`).
  Stop it before running the binary by hand — two instances fight over `/dev/fb0`, the
  GPIO lines, and SPI: `ssh ramp 'sudo systemctl stop bibby'`, then
  `ssh ramp 'sudo systemctl start bibby'` when done.
- To launch something long-running without hanging the ssh session, wrap it in a
  subshell: `ssh ramp '(setsid ./build/bibby >log 2>&1 </dev/null &)'`
- Logs: `ssh ramp 'journalctl -u bibby -n 100 --no-pager'` for the service; CSV run logs
  land under `~/bibby/logs/` and are readable straight from the Mac over the mount.
- Before trusting file edits, sanity-check the mount — if `ls /Users/austin/bibby` looks
  empty, the mount dropped; wait for the agent or edit via `ssh ramp` + heredoc.
- Git: the Pi is already authenticated to GitHub. Pushing from the Mac works but reads
  every object across sshfs and is slow; `ssh ramp 'cd ~/bibby && git push'` is quicker.

## Safety
- A  safe and robust system is important as this is dealing with high power and high voltage.
- The SSR drive shall be low whenever the controller is starting up, exiting, or has crashed.
  Held by: GPIO lines requested `INACTIVE` before any thread runs, the ssr thread driving both
  lines low on its way out, and the kernel releasing the lines (SoC pull-downs low) on any
  death. `bibby.service` restarts the process if it dies mid-brew.
- If the control loop stops servicing the actuator, the outputs to the SSRs shall be low. Held
  by: the control-staleness watchdog in the ssr thread (heartbeat frozen for ~2 s → duties
  forced to 0). Exercise it with `BIBBY_TEST_WEDGE`.
- If mains sense is lost, the outputs shall be low: the ZC watchdog turns both SSRs off and
  raises `watchdog_alarm`.
- When the temperature is not reliable — any RTD fault bit, a silent DRDY, or no valid sample
  yet — automatic control may not command power; the sampler forces manual at 0 W, sets
  `fault_forced_manual`, and the UI shows why. Only manual control may drive the SSRs then.
- One instance only: the flock guard keeps a second `bibby` from fighting over the SSR lines.

## Configurability
- There shall be an ini file that configures key parameters that another person/setp would need.
- Items for configuration include, 50Hz/60Hz, wattage per element, surface area of element per element, PID parameters.
- All configurable items should be in the ini file. There should not be a circumstance for someone to have to change a constant in the code and have to recompile to use.


## Documentation
- This project should be documented in a way that someone else can reproduce this project.
- The read me for this project should include everything, including the enclosure, electrical diagram, raspberry pi hat PCB, bring up and installation of raspberry pi softare, and methodology of calibrating control parameters for a future brewing set up.
- The documentation should include diagrams and architecture of how the software works.
- As updates are made, ensure that the readme has up-to-date information and includes no missing features. The readme should have a good flow for someone not familiar with the project to become familiar, good structure in documentation.
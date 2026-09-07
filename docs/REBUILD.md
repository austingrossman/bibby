\# bibby — Rebuild Specification (fresh start)

This is the one document needed to rebuild **bibby**, the brew-in-a-bag (BIAB)
temperature/power controller, from scratch with new code. It is written for an
engineer or AI starting cold. It states what to keep, what to change, and why.

The existing repository is still present and can be read for reference. The
**hardware is fixed** (the Pi HAT PCB, pinout, and sensor are not changing), and
several software pieces are proven and should be carried over close to
verbatim. Everything else is open to a cleaner design. This document calls out
each case.

Read it top to bottom once before writing any code.

---

## 0. Ground rules for the rewrite

These come from the project owner and override taste:

- **Language:** C by preference. C++ only where a library forces it, and then
  plain C++ — no templates, no clever generics. Never make the reader guess a
  type that isn't meant to be dynamic. Use fixed, explicit types.
- **Units:** SI / engineering units everywhere, including inside the control
  loop. Temperature in **°C**, power in **watts**, area in **cm²**, time in
  **seconds**. The display additionally shows temperature in **°F and °C**. Duty
  fraction `[0,1]` exists only at the actuator boundary (the SSR), not in the
  control law. (See §5 for the one deliberate unit choice.)
- **Structure:** small files, small functions, one responsibility each. No
  source file that crams several concerns together. Avoid abstraction that isn't
  paying for itself. If a function fits on a screen and reads top-to-bottom,
  prefer that over a framework.
- **Safety is the point.** This switches mains into multi-kilowatt elements. The
  default state of everything is **off**. Every safety mechanism in §4 is a
  requirement, not a feature.
- **2-space indent. `//` comments in C. `/* ... */` used when commenting a large block **

---

## 1. What the system does

Reads kettle temperature from a PT100 RTD, runs a temperature controller, and
delivers power to two mains heating elements through zero-cross solid-state
relays (SSRs). Power is modulated as a **duty cycle synchronized to the AC mains
zero crossing** (fire whole half-cycles, never phase-angle chop). A touchscreen
shows temperature and lets the user set a target, run manual or automatic
control, and watch the control behavior. Everything is logged to CSV for tuning.

Default target system: **BrewHardware "Premium Recirculating Electric (240 V)
BIAB Package — Get Tanked Edition, 20 gal"**, two heating elements, recirculated
(pumped) so the kettle is well mixed. The two-element, low-watt-density design
is the reason the power-split law in §6 matters.

---

## 2. Hardware — fixed, do not redesign

The Pi HAT PCB (`pi_hat/bibby_pi_hat/`, KiCad) and the pinout are **unchanged**.
Treat them as given. Read the schematic for the authoritative net list; the
sensor datasheet is at `docs/MAX31865.pdf`.

**Platform:** Raspberry Pi 5, 64-bit Raspberry Pi OS. GPIO chip is
`/dev/gpiochip4`. SPI is `/dev/spidev0.0`.

**Pin map (40-pin header):**

| Pin | GPIO | Signal | Dir | Role |
|---|---|---|---|---|
| 1 | — | 3.3 V | pwr | MAX31865 VIN |
| 6 | — | GND | pwr | MAX31865 GND |
| 19 | GPIO10 | SPI0 MOSI | out | MAX31865 SDI |
| 21 | GPIO9 | SPI0 MISO | in | MAX31865 SDO |
| 23 | GPIO11 | SPI0 SCLK | out | MAX31865 CLK |
| 24 | GPIO8 | SPI0 CE0 | out | MAX31865 CS |
| 36 | GPIO16 | DRDY | in | MAX31865 data-ready, active-low |
| 37 | GPIO26 | SSR2 | out | element 2 SSR drive |
| 38 | GPIO20 | Zero-cross | in | mains ZC pulse (rising edge) |
| 40 | GPIO21 | SSR1 | out | element 1 SSR drive |

**Front-end blocks on the HAT:**
- **RTD:** MAX31865 (PT100, 3-wire), 400 Ω 0.1 % reference resistor `Rref`
  (`R1`). Precision of `Rref` sets measurement accuracy and is a calibration
  anchor (§8).
- **Zero-cross:** H11AA1 AC-input optocoupler (anti-parallel LED — conducts on
  both half-cycles) with an RC/bias network; its output pulses at each mains zero
  crossing into GPIO20. Galvanically isolates mains sense from Pi logic.

**Safety-relevant hardware fact (verify before trusting §4):** GPIO21 and GPIO26
drive the SSR inputs directly — there is no external pull on those nets on the
HAT. On the Pi 5 these lines power up with the SoC **internal pull-downs**, so
before any software runs, and after the process releases the lines (normal exit
or crash), the SSR inputs read low → **elements off**. The whole single-process
safety argument leans on this. Confirm it on the bench (§11) and, if you ever
respin the HAT, add explicit external pull-downs on both SSR gates.

---

## 3. Architecture — one process, three concurrency domains

The previous build used **two processes** (a C relay controller and a C++
GUI/sensor frontend) talking through POSIX shared memory, specifically to keep
the safety-critical relay code isolated from the large GUI. That is allowed to
change. The owner's requirement is not "two processes" — it is that **three
activities run independently and none stalls the others**:

1. The **SSR modulator** runs off the zero-crossing edge, on its own timing.
2. The **RTD sampler** runs off the sensor's data-ready (DRDY) signal, on its
   own timing (edge interrupt or poll).
3. The **GUI** refreshes at its own rate, inhibited by neither of the above.

**Recommendation: a single binary with three threads.** This removes the shared
memory, the two-binary launch orchestration, the cross-process heartbeat, and an
entire class of "is the other process alive" logic — a large simplification that
the owner explicitly prioritizes. The isolation the two-process split bought is
replaced by the mechanisms in §4, which are sufficient because **process death
is already a safe state** (§2: lines released → pull-down → off).

```
  +--------------------------- bibby (one process) ----------------------------+
  |                                                                            |
  |  Thread A: SSR modulator          Thread B: RTD sampler     Main: GUI      |
  |  (real-time, SCHED_FIFO)          (blocks on DRDY edge)     (vsync-paced)   |
  |                                                                            |
  |  wait ZC edge (GPIO20) --------.  wait DRDY edge (GPIO16)   render frame    |
  |  sigma-delta -> fire SSR1/2    |  read MAX31865 -> °C       read snapshot   |
  |  ZC watchdog (no edge->off)    |  filter -> T_filt          draw UI + graph |
  |  control-stale watchdog        |  control law -> P_demand   handle touch    |
  |                                |  power split -> d1,d2       write setpoint, |
  |                                |  log row (rate-limited)     mode, manual   |
  |         ^        |             |        |        ^                 |        |
  |         |        v             |        v        |                 v        |
  |     [ duty1,duty2, heartbeat ]<+--------'   [ T, PID terms,   [ setpoint,   |
  |     [ output1/2, wdog, zc# ]---+----------> history buffer ]   mode, u_man, |
  |          in-process state (atomics + one mutex-guarded history)  sim_zc ]   |
  +----------------------------------------------------------------------------+
```

**Shared state** is a single in-process struct — the same idea as the old shared
memory, minus `shm_open`/`mmap`/`ftruncate` and minus IPC. Scalars are
`std::atomic` (or C11 `_Atomic`); the only compound shared object is the graph
history ring, guarded by one mutex. No lock-free queues, no message passing —
that would be over-abstraction for this data.

**Why threads, not processes, is safe enough here:**
- A crash in any thread ends the process; the OS releases the GPIO lines; the
  SSR inputs fall to their pull-down → off. Same end state the old two-process
  design relied on at boot and on double-fault.
- The SSR thread is tiny and runs at real-time priority (`SCHED_FIFO`), so GUI
  or GL stalls cannot delay a zero-cross response or the ZC watchdog.
- A **control-staleness watchdog** inside the SSR thread (see §4) forces duties
  to zero if thread B stops refreshing them — the in-process equivalent of the
  old cross-process frontend heartbeat.
- Optionally arm the Pi hardware watchdog (`/dev/watchdog`) petted from the SSR
  thread as a final backstop against a wedged process.

**Threading niceties (do these, they are cheap):**
- `mlockall(MCL_CURRENT|MCL_FUTURE)` at startup so the RT thread never page-faults.
- Set thread A to `SCHED_FIFO` at a modest priority; leave B and the GUI normal.
- Each thread opens the GPIO lines it owns (A: ZC + SSR1/2; B: DRDY). Different
  line offsets, so requests coexist on the one chip handle.

> **Note on a stale project memory.** An earlier note says "vsync off + 1 ms DRDY
> poll; don't re-enable vsync single-threaded." That constraint was a
> consequence of sampling on the single GUI thread. With sampling moved to its
> own thread (B) blocking on the DRDY edge, **the GUI can and should use vsync
> normally** — no busy-poll, no tearing, lower CPU. The old note does not apply
> to this architecture.

---

## 4. Safety requirements → mechanisms

Every row is a hard requirement. Implement all of them.

| Requirement | Mechanism |
|---|---|
| Elements **off at startup** | Drive both SSR GPIOs low as the first action, before the modulator runs. Duty state initializes to 0. |
| Elements **off on exit or crash** | `SIGINT`/`SIGTERM` handler drives SSRs low then exits; on any crash the OS releases the lines → pull-down → off (§2). |
| Elements **off if the mains-sense (ZC) signal is lost** | ZC watchdog in thread A: if no zero-crossing edge arrives within one mains half-period + 0.7 ms guard (≈9.0 ms @60 Hz, ≈10.7 ms @50 Hz), force both SSRs off and raise a `watchdog_alarm` flag. |
| Elements **off if control stops updating** | Control-staleness watchdog in thread A: thread B bumps a heartbeat counter each control update; if it does not advance for ~2 s worth of zero crossings, thread A forces duties to 0. |
| **No automatic temp control on unreliable temperature** | If the RTD reports any fault, force **manual** mode so the automatic control path cannot command power on bad data. Automatic mode is only reachable with a healthy sensor. |
| **Bounded commands** | Clamp per-element duty to `[0,1]` at the SSR boundary and clamp `P_demand` to `[0, P1+P2]` in the control path. |
| **One instance only** | `flock` single-instance guard on a lockfile at startup (kernel releases it on any exit). One binary now, so one lock. |
| **Bench testing without mains** | A `simulate_zc` flag makes thread A treat its ZC-watchdog timeout as a zero crossing, so the modulator and UI run with no AC connected. |

**Zero-cross simulation caveat:** when `simulate_zc` is set, the ZC-lost watchdog
must be suspended (a simulated edge is, by definition, "no real edge"). Keep this
path obviously separate from the live path so it can never mask a real ZC loss
during a brew. The visibility/enablability of the simulate ZC button will be in the INI config, so it can be removed after initial hardware checkout.

---

## 5. The one deliberate unit choice: watts inside the loop

Keep the control law in engineering units. The controller consumes temperature
in **°C** and produces a **power demand in watts** `P_demand ∈ [0, P1+P2]`. This
is physically clean:

```
  Kp [W/°C] · error [°C] = [W]           proportional term is a power
  feedforward [W]                        holding power to fight heat loss
  process gain K [°C/W] = 1/h            identified from a step test (§8)
```

Duty fraction `[0,1]` appears only downstream, at the SSR, where it belongs. The
`[0,1]` value is an actuator command, not a control quantity, so it does not
live in the loop. This satisfies "engineering units even inside the PID."

The one place `[0,1]` is unavoidable is the sigma-delta modulator, which by
definition schedules a *fraction* of half-cycles. That is a hardware detail
below the control law, not part of it.

---

## 6. Control chain, end to end

```
 RTD ─► MAX31865 ─► CVD+cal ─► filter ─► control law ─► power split ─► sigma-delta ─► SSRs
        (§7)        (°C)       (°C)      (°C ➜ W)        (W ➜ d1,d2)    (d ➜ fired)    (§7)
```

Stage by stage:

1. **MAX31865 → °C** (§7): non-blocking read, Callendar–Van Dusen, per-unit
   calibration. Carry over from the existing driver.
2. **Filter:** the existing `SecondOrderAverage` (two cascaded N-sample boxcars
   = triangular-weighted CIC) is fine; keep it. Primes on the first sample so it
   starts at the true value. Group delay ≈ `N/f_s` feeds into the dead-time term
   for tuning (§8).
3. **Control law (°C → W):** PID on `error = setpoint − T_filt`, run once per
   fresh sample, output clamped to `[0, P1+P2]` watts. Add an optional
   **holding feedforward** in watts: `u_ff = (setpoint − ambient) / K`, the
   steady-state power to hold the setpoint against heat loss, so feedback only
   trims model error. Anti-windup by **conditional integration** (hold the
   integrator when the output is already railed and the error pushes further into
   the rail). This mirrors the proven logic in the old `pid.cpp`; only the units
   change (duty → watts) and the clamp bound becomes `P1+P2`. 
   In addition, there is a second PID mode "grain in." When the grain is in, the PID will have a different set of coefficients, and the heating elements have a lower peak output wattage.
4. **Power split (W → per-element duty):** §6.1. This is the new intermediate
   function the owner asked for.
5. **Sigma-delta (duty → fired half-cycles):** §7. Carry over unchanged.

Manual mode bypasses stages 3–4: the slider commands a power (or a duty) that
feeds the split / the SSRs directly.

### 6.1 Power split — minimize surface power density (anti-scorch)

Two elements with rated power `P1,P2` (W) and wetted area `A1,A2` (cm²). Given a
demanded power `P_demand`, choose per-element power `x1,x2` (then duty
`di = xi/Pi`) to **minimize the peak surface power density** `xi/Ai` — high flux
is what scorches wort onto the element.

Minimizing the maximum of `x1/A1, x2/A2` subject to `x1+x2 = P_demand` puts both
fluxes **equal** (spread the load over all available area):

```
  x_i = P_demand · A_i / (A1 + A2)        // equal flux on both elements
  d_i = x_i / P_i
```

Then handle saturation (one element hitting `di = 1` before the other):

```
  A  = A1 + A2
  x1 = P_demand * A1 / A
  x2 = P_demand * A2 / A
  if (x1 > P1) { x2 += x1 - P1; x1 = P1; }   // push overflow to the other
  if (x2 > P2) { x1 += x2 - P2; x2 = P2; }
  x1 = clamp(x1, 0, P1);  x2 = clamp(x2, 0, P2);
  d1 = x1 / P1;  d2 = x2 / P2;
```

Properties worth knowing:
- For **identical elements** (`P1=P2, A1=A2`) this gives `d1 = d2 =
  P_demand/(P1+P2)` — i.e. both elements at the same duty, which is exactly what
  the old code did. The new law is the general case that also handles unequal
  elements correctly.
- Total deliverable power is `P1+P2`; clamp `P_demand` there.
- **Optional scorch cap:** add `element.max_flux_w_cm2` to config. If set, cap
  `P_demand ≤ max_flux · (A1+A2)` so the nominal (equal-flux) case never exceeds
  the limit. BrewHardware's ultra-low-watt-density elements exist precisely to
  keep flux low; a configurable cap in W/cm² lets the controller honor that in
  engineering units. Keep it optional (0 = disabled).

Put this in its own file (`power_split.{h,c}`), pure function, no state, trivially
unit-testable.

---

## 7. Modules to carry over (proven — port, don't reinvent)

Port these with minimal changes. They are correct and were validated on the
hardware. However, restructuring to make code cleaner is encouraged.

**MAX31865 driver** (`sensors/max31865`):
- SPI `/dev/spidev0.0`, 1 MHz, `SPI_MODE_1`. PT100, 3-wire, 400 Ω `Rref`.
- Init order (datasheet-mandated): write config with 3-wire + fault-clear +
  **mains-notch filter bit** (bit0: set=50 Hz, clear=60 Hz) → enable VBIAS, wait
  10 ms → enable auto-conversion → discard the first couple of conversions using
  DRDY. The notch bit must be set **before** auto-conversion.
- `read_temperature()` is **non-blocking**: check DRDY; if not ready return the
  last value flagged not-fresh. If ready, burst-read RTD + fault registers,
  convert via Callendar–Van Dusen, apply per-unit gain/offset (§8), return fresh.
- **Fault handling:** check the RTD-LSB fault bit and the Fault Status register
  every read; reject a faulted conversion (hold the last good value so the filter
  is not poisoned), clear the latch, and expose a decoded fault string for the UI.
- In the new design, thread B can **block on the DRDY falling edge** (libgpiod v2
  edge event) instead of polling — cleaner and it decouples sampling from the GUI
  entirely. Keep the non-blocking `read_temperature` too; it is still useful.

**Sigma-delta ZC modulator** (into thread A): each zero crossing, add the
per-channel duty to an accumulator; when it reaches 1.0, fire that SSR for the
half-cycle and subtract 1.0; cap the accumulator at 2.0 to bound catch-up. This
is a first-order delta-sigma DAC clocked by the mains — average fired fraction
equals commanded duty, switching energy pushed to high frequency. Carry over
verbatim.

**Temperature filter** (`temp_filter`): keep structure of `SecondOrderAverage` default as-is. Equivalent to a second order CIC filter. Have filter filter order and length be options in the INI.

**INI config loader** (`config`): keep the tiny section/key parser, defaults, and
`$BIBBY_CONFIG` lookup. Extend the key set (§9).

**Single-instance flock** (`single_instance`): keep.

**Calibration methodology** (README §8/§9 of the old README): the ice-point
`Rref` trim and two-point span gain/offset procedure are sound and per-unit.
Carry the method over; the only change is that the identified **process gain K is
now °C/W** (= 1/h), not °C per unit duty (§8). Note: please evaluate whether a new calibration method is better.

---

## 8. Calibration & tuning (methodology carries over, units change)

This has not been actually verified and ran with real hardware. Evaluate if this is the best method.

**Sensor calibration** (per probe + board, unchanged in method):
1. `Rref` ice-point one-point trim: `Rref = Rref_nom · 100.0 / R_pt100(T_ice)`.
2. Two-point span: `T_true = gain·T + offset` from two reference points (ice
   point and altitude-corrected boiling point are the cheap, wide-span choices).
All three live in `[sensor]` in the INI; identity defaults leave the sensor
uncalibrated but functional. (Full worked procedure is in the old README §9 —
reuse it.)

**Plant model & PID tuning** — same open-loop step-test method, re-expressed in
watts:
- The plant is well approximated as First-Order-Plus-Dead-Time:
  `G(s) = K·e^(−Ls)/(τs+1)` with `K` in **°C/W**, `τ` the kettle time constant
  (tens of minutes for this batch size), `L` the effective dead time (probe lag
  + filter group delay). `L/τ` is small for a big kettle — the challenge is the
  slow time constant, not dead time.
- `tools/identify_plant.py` fits `K, τ, L` from a constant-power step and prints
  PID gains. **Update it** so:
  - the step power it fits against is `duty · (P1+P2)` in watts (it already knows
    the duty and can read the wattages from the INI),
  - it reports `K` in °C/W and gains in W-based units (`Kp` in W/°C, etc.),
  - it prints a ready-to-paste `[feedforward] process_gain_c` (= K, °C/W) and a
    suggested `ambient_c`.
- Prefer **IMC-PI** with `λ ≈ L` (or larger for a mash — overshoot costs enzyme
  activity). ZN/Cohen-Coon offered as alternatives.
- The controller accumulates raw error per sample (`integral += error`, deriv =
  `Δerror`), so the discrete INI gains fold in the sample period `T_s`:
  `ki_ini = Ki_c·T_s`, `kd_ini = Kd_c/T_s`, `kp_ini = Kp_c`. `T_s` is recovered
  from the log's monotonic timestamp column. Keep this convention documented.

---

## 9. Configuration (INI) — one file, no recompiles

Everything another person/setup would change stays in `bibby.ini`, loaded at
startup. Carry the old keys, add the watts/logging ones. Missing keys fall back
to safe defaults (elements off-capable, PID gains zero → auto commands zero
power until tuned).

```ini
[mains]
frequency_hz = 60            ; 50 or 60; sets ZC watchdog timeout + sensor notch

[element1]                   ; SSR1, GPIO21 / pin 40
watts    = 5000              ; rated power, W  — CONFIRM against your package
area_cm2 = 820               ; wetted surface area, cm² — MEASURE/estimate
[element2]                   ; SSR2, GPIO26 / pin 37
watts    = 5500
area_cm2 = 473

[power]
max_flux_w_cm2 = 0.0         ; optional anti-scorch cap; 0 = disabled (§6.1)

[pid]                        ; control law; output is WATTS now (§5)
kp = 0.0                     ; W/°C   (zero until tuned)
ki = 0.0
kd = 0.0

[feedforward]                ; holding power feedforward (§6, §8)
process_gain_c = 0.0         ; identified K, °C/W; 0 disables feedforward
ambient_c      = 20.0        ; cold-start/room reference, °C

[sensor]                     ; per-unit RTD calibration (§8)
ref_resistor_ohms = 400.0    ; MAX31865 Rref, ice-point trimmed
temp_cal_gain     = 1.0
temp_cal_offset   = 0.0

[logging]
rate = high                  ; "high" = every sample (tuning); "low" = decimated
low_period_s = 2.0           ; row interval when rate=low
```

> The default element wattage/area above are **placeholders to confirm** against
> the actual Get Tanked package and a physical measurement — the control math is
> independent of the exact numbers, but the feedforward and flux cap are not.

The mains frequency still drives: the ZC-watchdog timeout (half-period + 0.7 ms
guard), the control-staleness count (~2 s of ZC), and the MAX31865 notch bit.

---

## 10. UI

**Requirement:** touchscreen, mounted sideways (portrait panel, landscape use —
the old build rendered portrait then rotated 90° CW and remapped touch). The GUI
must refresh independently of sampling and modulation (satisfied by §3: it is its
own thread). ImGui is **not** required.

**Recommendation: LVGL** for the rewrite, with the proven **SDL3 + Dear ImGui +
GLES3** stack as the safe fallback.

- **LVGL** is a C library purpose-built for embedded touchscreen HMIs. It gives:
  native display **rotation** (a config flag — no hand-written FBO + rotation
  shader), native touch input, and retained-mode widgets including a **chart**
  widget that fits the PID graph (§10.1). Choosing it keeps the **entire
  application in C**, matching the owner's language preference and collapsing the
  C/C++ split. Backend on the Pi: LVGL over SDL (easy) or DRM/KMS + evdev.
- **ImGui fallback** is proven on this exact hardware, and the rotation +
  touch-remap are already solved in `frontend/display.cpp` and `main.cpp`. It is
  C++, and it needs the manual rotation. Use it if LVGL integration on the panel
  proves troublesome — the UI is not where the project's value is, so do not sink
  days into a toolkit.

Either way, keep UI code out of the control and safety paths. The UI only
**reads** a state snapshot and **writes** setpoint / mode / manual command /
`simulate_zc` / `grain_in`.

**Screen contents** (carry the old layout intent):
- Large readout: setpoint (°C) and measured temperature in **both °C and °F**.
- Element indicators that track each SSR's fired state.
- Full-height duty/power control, active only in manual mode. Have PID output update the display for this control (looks nice) even though it is disabled.
- Setpoint steppers (±10 / ±1 / ±0.1 °C).
- Toggles: **Manual Control**, **Test Touch** (simulate ZC, rename buttont o ZC Sim, don't display if INI config says to not display), **Grain In** (logged
  event marker).
- The PID graph (§10.1).
- A fault band: decoded RTD fault and watchdog alarm when active. When a fault
  forces manual mode, say so on screen.

### 10.1 The PID graph — plot more than temperature

For tuning you need to *see the loop*, not just the temperature. Plot, sharing a
time axis (last few minutes, scrolling):

- **Setpoint** and **filtered temperature** (and optionally **raw** temperature,
  lighter, to judge noise vs. filter lag).
- **Error** (`setpoint − T`) — the quantity the loop actually acts on.
- **Total commanded power** `P_demand` (W), and the **per-element duties** `d1,
  d2` — so you can watch the split and saturation.
- **PID term breakdown:** feedforward, P, I, D contributions (in W). This is the
  single most useful tuning view — it shows whether the integrator is doing the
  holding work (it should not, if feedforward is sized right), whether D is just
  amplifying noise, and how the terms trade off during a transient.
- **SSR fired state / actual delivered power**, to confirm the modulator is
  tracking the command.
- **Event markers:** grain-in, mode changes, setpoint changes — align brew events
  to the trace.

Practical: a two-pane graph (temperature/setpoint on top, the power/term
breakdown below, shared X) reads better than one crowded axis. LVGL's chart or
an ImGui draw-list plot both do this. Feed it from the same per-sample data the
logger writes, so the on-screen graph and the CSV never disagree.

### 10.2 Manual / auto and the fault interlock

- **Manual:** the on-screen control drives the elements (through the §6.1 split
  or directly). Always available.
- **Auto:** the control law drives them. Reachable **only** with a healthy
  sensor; any RTD fault forces manual (§4) and the UI shows why.

---

## 11. Logging — high rate and low rate

One timestamped CSV per run, `~/bibby/logs/YYYY/MM/DD/HH-MM-SS.csv`, directories
auto-created, each row `fflush`ed so a crash keeps the data. Written from thread
B, which has the fresh sample and the current control terms.

**Two rates** (owner requirement):
- **High** — one row per fresh sample (~50–60 Hz). Used while **tuning** and for
  the plant-ID step test; nothing is decimated.
- **Low** — decimated to `low_period_s` (default ~1–2 s). Used for **normal
  brewing** after tuning, to keep files small over a multi-hour brew.

Selected by `[logging] rate` in the INI. The user should use high for tuning, flip to low once dialed in). Implement as a
simple time-based decimator on the write path — same row format either way, only
the interval changes. Do not fork the schema.

**Columns** (superset of the old set, in engineering units):

```
wall_time, t_monotonic_s, temp_raw_c, temp_filt_c, setpoint_c,
p_demand_w, duty1, duty2, flux1_w_cm2, flux2_w_cm2,
pid_ff_w, pid_p_w, pid_i_w, pid_d_w, pid_integral, pid_deriv,
pid_error_c, manual, grain_in, rtd_fault, watchdog
```

- `wall_time` ISO-8601 local w/ ms; `t_monotonic_s` a monotonic clock — recover
  `dt` from either.
- Keep the P/I/D/feedforward breakdown; it is what §8 tuning consumes.
- `flux*_w_cm2` (= `d_i·P_i/A_i`) make the anti-scorch behavior directly visible.

---

## 12. Suggested file layout

Small, single-purpose files. One binary. Illustrative, not prescriptive:

```
bibby/
├─ bibby.ini
├─ src/
│  ├─ main.c                 startup: config, threads, safe-state, teardown
│  ├─ state.h                shared in-process state (atomics) + history mutex
│  ├─ ssr_thread.c/.h        thread A: ZC edge, sigma-delta, both watchdogs
│  ├─ sampler_thread.c/.h    thread B: DRDY edge, read, filter, control, split, log
│  ├─ control.c/.h           PID (°C → W), feedforward, anti-windup
│  ├─ power_split.c/.h       W → (d1,d2), equal-flux + optional flux cap (§6.1)
│  ├─ sigma_delta.c/.h       duty → fired half-cycle decision
│  ├─ max31865.c/.h          RTD driver (ported)
│  ├─ temp_filter.c/.h       SecondOrderAverage (ported)
│  ├─ csv_logger.c/.h        dual-rate CSV (§11)
│  ├─ config.c/.h            INI loader (ported + new keys)
│  ├─ single_instance.h      flock guard (ported)
│  └─ ui/                    LVGL (or ImGui) UI, isolated from control
├─ tools/identify_plant.py   plant ID + tuning (watts update, §8)
├─ pi_hat/                   UNCHANGED KiCad project
└─ docs/MAX31865.pdf
```

If the UI stays ImGui/C++, only the `ui/` subtree and its objects are C++; the
whole control/safety core stays C.

---

## 13. Build & deploy

- **CMake**, out-of-source. One executable. Link `gpiod` and `rt`; plus the UI
  toolkit (LVGL, or SDL3-static + GLESv2 for the ImGui fallback).
- `libgpiod v2` is required (v2 edge-event API). Enable SPI (`/dev/spidev0.0`).
- Keep `build/` and `logs/` gitignored; keep `bibby.ini` checked in.
- Make the config path and any asset paths **runtime-resolvable** (resolve
  `/proc/self/exe`, look next to the binary, honor `$BIBBY_CONFIG`) so an
  installed binary works — do not bake build-tree paths as the old build did.

**Autostart at boot (systemd).** One binary now, so one unit. The RT thread needs
scheduling privilege and the process needs GPIO/SPI access.

```ini
# /etc/systemd/system/bibby.service
[Unit]
Description=bibby brew controller
After=multi-user.target

[Service]
User=ramp
SupplementaryGroups=gpio spi video render input
AmbientCapabilities=CAP_SYS_NICE          # allow SCHED_FIFO on the SSR thread
Environment=BIBBY_CONFIG=/home/ramp/bibby/bibby.ini
ExecStart=/home/ramp/bibby/build/bibby
Restart=on-failure
RestartSec=3

[Install]
WantedBy=multi-user.target
```

```
sudo systemctl daemon-reload
sudo systemctl enable --now bibby
journalctl -u bibby -f
```

`CAP_SYS_NICE` (or a matching `LimitRTPRIO`) lets the SSR thread take real-time
priority. On exit/kill, the process drives SSRs low and the OS releases the lines
regardless (§4). If you enable the hardware watchdog, make sure the unit's
restart policy and the watchdog timeout are consistent.

---

## 14. Bring-up & test plan

Do these in order; do not connect mains until the bench tests pass.

1. **Build & run headless-safe.** Confirm single-instance lock, config load, and
   that SSR GPIOs read low before and during startup (meter the SSR gate lines).
2. **Sensor.** Verify DRDY toggling and sane °C in air; unplug an RTD wire and
   confirm the fault is detected, the value is held, and the UI shows the decoded
   fault. Confirm a fault **forces manual mode**.
3. **ZC simulation.** With **no mains**, enable `simulate_zc`; confirm the
   modulator fires the SSR gate lines at commanded duty (scope or LED), and that
   the ZC-lost watchdog stays *quiet* while simulating and *trips* (SSRs off,
   alarm) when you disable simulation with no real ZC present.
4. **Control-staleness watchdog.** Pause thread B (or stub it) and confirm thread
   A zeroes the duties within ~2 s.
5. **Crash-safety.** `kill -9` the process mid-fire; confirm the SSR gates fall
   low immediately (this is the pull-down path from §2 — verify it physically).
6. **Live, low power.** Only now connect mains through properly rated SSRs,
   fusing, and earth bonding. Verify real zero crossings drive `iteration`,
   run a low manual duty, watch the temperature respond.
7. **Plant ID → tune.** Run the §8 step test at high log rate, fit with
   `identify_plant.py`, paste gains + feedforward, validate in auto, then switch
   logging to low rate for real brews.

---

## 15. Decisions to confirm before coding

Most choices here are made and justified. Three are worth an explicit yes/no from
the owner, because they shape the whole rewrite:

1. **Single process, three threads** (§3) instead of two processes + shared
   memory. Recommended for simplicity; safety is preserved by §4 and the
   pull-down-off property (§2). Confirm §2's pull-down on the bench early — the
   argument depends on it.
2. **UI toolkit: LVGL** (all-C, native rotation/touch/chart) vs. keeping the
   proven **SDL3+ImGui** stack (§10). Recommended LVGL; ImGui is the low-risk
   fallback.
3. **Control output in watts** with a downstream power-split (§5, §6). Recommended
   for SI-unit cleanliness and correct handling of unequal elements; it changes
   the PID gain units and a few lines of `identify_plant.py`.

Everything else in this document is a direct port or a stated requirement.
```

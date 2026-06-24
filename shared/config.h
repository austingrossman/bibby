#pragma once
#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

// User-editable runtime configuration, loaded from an INI file. This is the one
// source of truth for the parameters another person or another brew setup would
// need to change: mains frequency, per-element electrical/physical specs, and
// PID gains. Both processes load the same file at startup.
typedef struct {
  int   mains_hz;            // AC line frequency, 50 or 60
  float element1_watts;      // SSR1 heating element rated power, W
  float element2_watts;      // SSR2 heating element rated power, W
  float element1_area_cm2;   // SSR1 element wetted surface area, cm^2
  float element2_area_cm2;   // SSR2 element wetted surface area, cm^2
  float pid_kp;              // PID proportional gain
  float pid_ki;              // PID integral gain
  float pid_kd;              // PID derivative gain
  // Holding feedforward: the steady-state duty to maintain the setpoint is
  // u_ff = (setpoint - ambient_c) / process_gain_c, added ahead of the PID so
  // the feedback only trims model error. process_gain_c is the identified
  // process gain K (deg C per unit duty, from tools/identify_plant.py). A zero
  // process_gain_c disables feedforward (the safe default). See README §9.6.
  float ff_process_gain_c;  // K, deg C per unit duty; 0 disables feedforward
  float ff_ambient_c;       // ambient/reference temperature, deg C
  // Per-unit RTD calibration (depends on the specific probe + reference
  // resistor). Identity defaults (400 Ohm, gain 1, offset 0) leave the sensor
  // uncalibrated; the as-built trims live in bibby.ini. See README §9.
  float sensor_ref_resistor_ohms;  // MAX31865 Rref, ice-point trimmed
  float sensor_temp_cal_gain;      // two-point span gain on the CVD output
  float sensor_temp_cal_offset;    // two-point span offset, °C
} BibbyConfig;

// Fill cfg with built-in defaults (safe: PID gains zero, 60 Hz), then overlay
// any recognized keys found in the INI file. When `path` is NULL/empty the
// lookup falls back to the $BIBBY_CONFIG env var, then the compile-time default
// (the BIBBY_CONFIG macro).
// A missing file or unknown keys are ignored (defaults stand). Returns true if
// a file was read, false if none was found.
bool config_load(BibbyConfig *cfg, const char *path);

// Nanoseconds with no zero crossing before the heater watchdog trips: one mains
// half-period (the nominal ZC spacing) plus a fixed guard band for jitter.
uint64_t config_zc_timeout_ns(const BibbyConfig *cfg);

// Zero-crossing count (~2 s of mains) before a stale frontend forces duty to 0.
uint32_t config_frontend_watchdog_zc(const BibbyConfig *cfg);

#ifdef __cplusplus
}
#endif

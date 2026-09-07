#pragma once
#include <stdbool.h>
#include <stdint.h>

// User-editable runtime configuration, loaded from bibby.ini at startup. This
// is the one source of truth for everything another person or another brew
// setup would change; nothing here should ever require a recompile.
//
// All values are SI / engineering units: watts, cm^2, degrees C, seconds.
// The PID gains are watts-based (kp in W/degC); see README tuning sections.
typedef struct {
  // [mains] — drives the ZC watchdog timeout, the control-staleness count,
  // and the MAX31865 noise-rejection notch. 50 or 60.
  int   mains_hz;

  // [element1]/[element2] — rated power and wetted surface area per element.
  // Used by the equal-flux power split and the anti-scorch cap.
  float element1_watts;
  float element2_watts;
  float element1_area_cm2;
  float element2_area_cm2;

  // [power] — optional anti-scorch cap on surface power density. When > 0,
  // total demand is capped at max_flux_w_cm2 * (A1 + A2). 0 disables.
  float max_flux_w_cm2;

  // [pid] — water-only gains. kp in W/degC; ki and kd are per-sample discrete
  // gains folded with the sample period (see tools/identify_plant.py).
  float pid_kp;
  float pid_ki;
  float pid_kd;

  // [grain] — gains used while the Grain In toggle is set, plus a lower peak
  // power for the loaded kettle. grain_max_power_w = 0 means no extra cap.
  float grain_kp;
  float grain_ki;
  float grain_kd;
  float grain_max_power_w;

  // [feedforward] — holding power u_ff = (setpoint - ambient_c) / process_gain_c
  // in watts, added ahead of the PID. process_gain_c is the identified process
  // gain K in degC/W; 0 disables feedforward (the safe default).
  float ff_process_gain_c;
  float ff_ambient_c;

  // [sensor] — per-unit RTD calibration (probe + board specific): ice-point
  // trimmed Rref, then T_true = gain*T + offset on the CVD output.
  float sensor_ref_resistor_ohms;
  float sensor_temp_cal_gain;
  float sensor_temp_cal_offset;

  // [filter] — cascaded boxcar temperature filter: `order` stages of `window`
  // samples each. Group delay ~= order * window / (2 * sample_rate).
  int   filter_order;
  int   filter_window;

  // [logging] — high = one CSV row per fresh sample (~50/60 Hz, for tuning);
  // low = one row per low_period_s (for normal brews).
  bool  log_high_rate;
  float log_low_period_s;

  // [ui] — show_zc_sim hides the ZC Sim button after hardware checkout.
  // chart_window_min is the scrolling graph span. fb_device/touch_device are
  // device paths, or "auto" to probe. rotation (0/90/180/270) maps the UI onto
  // the sideways-mounted panel.
  bool  ui_show_zc_sim;
  float ui_chart_window_min;
  int   ui_rotation;
  char  ui_fb_device[64];
  char  ui_touch_device[64];

  // [adaptive] — online thermal-mass gain scheduling. The m*c estimate is
  // always computed and logged; only when enable=true does it scale kp/kd by
  // (mc_est / mc_ref_j_per_c), clamped to [scale_min, scale_max].
  bool  adaptive_enable;
  float adaptive_mc_ref_j_per_c;
  float adaptive_scale_min;
  float adaptive_scale_max;
} BibbyConfig;

// Fill cfg with safe built-in defaults (PID gains zero, 60 Hz), then overlay
// any recognized keys from the INI file. Lookup order for the file: `path`
// argument -> $BIBBY_CONFIG -> bibby.ini next to the executable -> ./bibby.ini.
// Missing file or unknown keys leave the defaults standing. Returns true if a
// file was read; config_resolved_path() reports which one.
bool config_load(BibbyConfig *cfg, const char *path);

// Path of the INI file the last config_load() read (or tried last), for logs.
const char *config_resolved_path(void);

// Nanoseconds with no zero crossing before the ZC watchdog trips: one mains
// half-period plus a 0.7 ms jitter guard (60 Hz -> ~9.0 ms, 50 Hz -> ~10.7 ms).
uint64_t config_zc_timeout_ns(const BibbyConfig *cfg);

// Zero-crossing count (~2 s of mains) with a frozen control heartbeat before
// the SSR thread forces both duties to zero.
uint32_t config_staleness_zc(const BibbyConfig *cfg);

// Total rated power P1 + P2 in watts — the control-law output clamp.
float config_total_watts(const BibbyConfig *cfg);

#pragma once

// Temperature control law: degrees C in, watts out.
//
// pid_update() runs once per fresh temperature sample and returns a power
// demand in [0, out_max_w] watts for the power split. Gains are watts-based
// (kp in W/degC); ki/kd are discrete per-sample gains folded with the sample
// period by tools/identify_plant.py (integral += error per sample, deriv =
// error delta per sample — the sample clock is the MAX31865 conversion rate,
// ~50/60 Hz).
//
// A holding feedforward in watts may be passed in ahead of the feedback terms
// (see control_feedforward_w); the feedback then only trims model error.
//
// Integral separation: when i_band_c > 0 the integrator only accumulates
// while |error| <= i_band_c and holds (keeps its value, neither grows nor
// resets) outside that band. The kettle is an integrating plant, so error
// accumulated during a long approach must be paid back as overshoot after the
// crossing; with the feedforward carrying the holding power the integrator
// has only a few tens of watts to trim, and there is nothing useful for it to
// learn while the temperature is still degrees away. i_band_c = 0 integrates
// always (the classic PI).

// Breakdown of the most recent update, for logging/tuning/chart.
typedef struct {
  float error_c;   // setpoint_c - temp_c
  float ff_w;      // feedforward passed in
  float p_w;       // kp * error
  float i_w;       // ki * integral
  float d_w;       // kd * deriv
  float integral;  // integrator state (raw error accumulator)
  float deriv;     // per-sample error delta
  float output_w;  // clamp(ff + p + i + d, 0, out_max_w)
} PidTerms;

typedef struct {
  float kp, ki, kd;
  float i_band_c;   // integral-separation band, degC; 0 = integrate always
  float integral;
  float prev_error;
  int   has_prev;
  PidTerms terms;
} Pid;

void pid_init(Pid *pid, float kp, float ki, float kd);

// Set the integral-separation band (degC, 0 = off). Kept across
// pid_set_gains(): the band is a property of the approach, not the gain set.
void pid_set_i_band(Pid *pid, float i_band_c);

// Clear the integrator and derivative history (on mode or gain-set change).
void pid_reset(Pid *pid);

// Swap the gain set in place (water <-> grain-in); resets the dynamic state.
void pid_set_gains(Pid *pid, float kp, float ki, float kd);

// One control step. gain_scale multiplies kp/kd (adaptive thermal-mass
// scheduling; pass 1.0 when disabled). Returns the power demand in watts.
float pid_update(Pid *pid, float temp_c, float setpoint_c, float ff_w,
                 float out_max_w, float gain_scale);

// Holding feedforward: steady-state watts to hold setpoint_c against heat
// loss, (setpoint - ambient) / K with K the process gain in degC/W. Returns 0
// when process_gain_c_per_w is 0 (feedforward disabled). Clamped to
// [0, out_max_w].
float control_feedforward_w(float setpoint_c, float ambient_c,
                            float process_gain_c_per_w, float out_max_w);

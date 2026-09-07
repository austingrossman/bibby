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
  float integral;
  float prev_error;
  int   has_prev;
  PidTerms terms;
} Pid;

void pid_init(Pid *pid, float kp, float ki, float kd);

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

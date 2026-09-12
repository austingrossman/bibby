#include "control.h"

#include <string.h>

static float clampf(float v, float lo, float hi) {
  return v < lo ? lo : (v > hi ? hi : v);
}

void pid_init(Pid *pid, float kp, float ki, float kd) {
  memset(pid, 0, sizeof(*pid));
  pid->kp = kp;
  pid->ki = ki;
  pid->kd = kd;
}

void pid_reset(Pid *pid) {
  pid->integral   = 0.0f;
  pid->prev_error = 0.0f;
  pid->has_prev   = 0;
}

void pid_set_i_band(Pid *pid, float i_band_c) {
  pid->i_band_c = i_band_c > 0.0f ? i_band_c : 0.0f;
}

void pid_set_gains(Pid *pid, float kp, float ki, float kd) {
  pid->kp = kp;
  pid->ki = ki;
  pid->kd = kd;
  pid_reset(pid);
}

float pid_update(Pid *pid, float temp_c, float setpoint_c, float ff_w,
                 float out_max_w, float gain_scale) {
  float kp = pid->kp * gain_scale;
  float kd = pid->kd * gain_scale;

  float error = setpoint_c - temp_c;
  float deriv = pid->has_prev ? (error - pid->prev_error) : 0.0f;
  pid->prev_error = error;
  pid->has_prev   = 1;

  float p = kp * error;
  float d = kd * deriv;

  // Conditional-integration anti-windup. Test the output the feedforward and
  // the non-integral terms already command: if it is at/over a rail and the
  // new error would push further into the same rail, hold the integrator.
  // This confines the integrator to the headroom the feedforward leaves.
  float out_pre_step = ff_w + p + pid->ki * pid->integral + d;
  int   sat_high     = out_pre_step >= out_max_w && error > 0.0f;
  int   sat_low      = out_pre_step <= 0.0f && error < 0.0f;
  // Integral separation: outside the band the integrator holds. See control.h.
  int   outside_band = pid->i_band_c > 0.0f &&
                       (error > pid->i_band_c || error < -pid->i_band_c);
  if (!sat_high && !sat_low && !outside_band) {
    pid->integral += error;
    // Backstop: bound the integral term to full output authority, so a long
    // saturated fault cannot wind it beyond anything the clamp can express.
    if (pid->ki > 0.0f) {
      float integral_max = out_max_w / pid->ki;
      pid->integral = clampf(pid->integral, -integral_max, integral_max);
    }
  }

  float i   = pid->ki * pid->integral;
  float out = clampf(ff_w + p + i + d, 0.0f, out_max_w);

  pid->terms.error_c  = error;
  pid->terms.ff_w     = ff_w;
  pid->terms.p_w      = p;
  pid->terms.i_w      = i;
  pid->terms.d_w      = d;
  pid->terms.integral = pid->integral;
  pid->terms.deriv    = deriv;
  pid->terms.output_w = out;
  return out;
}

float control_feedforward_w(float setpoint_c, float ambient_c,
                            float process_gain_c_per_w, float out_max_w) {
  if (process_gain_c_per_w <= 0.0f) return 0.0f;
  float ff = (setpoint_c - ambient_c) / process_gain_c_per_w;
  return clampf(ff, 0.0f, out_max_w);
}

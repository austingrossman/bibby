#include "pid.h"

// Hard backstop on the integrator. The primary anti-windup is conditional
// integration (below), which already keeps the integrator within the headroom
// the feedforward leaves. With a holding feedforward the integrator only trims
// model error, so it never needs much authority; this cap just bounds it if a
// fault leaves the loop saturated for a long time. See README §9.5/§9.6.
static constexpr float INTEGRAL_MAX = 1000.0f;

static float clampf(float v, float lo, float hi) {
  return v < lo ? lo : (v > hi ? hi : v);
}

void PidController::reset() {
  integral_   = 0.0f;
  prev_error_ = 0.0f;
  has_prev_   = false;
}

float PidController::update(float temp_c, float setpoint_c, float feedforward) {
  // One temperature sample per call, so the loop time step is implicit (one
  // sample). dt cancels into the gains for now; make it explicit if the sample
  // cadence is changed.
  float error = setpoint_c - temp_c;
  float deriv = has_prev_ ? (error - prev_error_) : 0.0f;
  prev_error_ = error;
  has_prev_   = true;

  float p = kp_ * error;
  float d = kd_ * deriv;

  // Conditional-integration anti-windup. Test the output that the feedforward
  // and the non-integral terms already command. If that is at/over a rail and
  // the new error would drive it further into the same rail, hold the
  // integrator; otherwise integrate. This lets the integrator wind only within
  // the [-feedforward, 1-feedforward] band the feedforward leaves available,
  // instead of fighting the feedforward bias.
  float out_pre_step = feedforward + p + ki_ * integral_ + d;
  bool  sat_high     = out_pre_step >= 1.0f && error > 0.0f;
  bool  sat_low      = out_pre_step <= 0.0f && error < 0.0f;
  if (!sat_high && !sat_low) {
    integral_ = clampf(integral_ + error, -INTEGRAL_MAX, INTEGRAL_MAX);
  }

  float i   = ki_ * integral_;
  float out = clampf(feedforward + p + i + d, 0.0f, 1.0f);

  terms_.error    = error;
  terms_.ff       = feedforward;
  terms_.p        = p;
  terms_.i        = i;
  terms_.d        = d;
  terms_.integral = integral_;
  terms_.deriv    = deriv;
  terms_.output   = out;
  return out;
}

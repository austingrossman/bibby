#include "pid.h"

// Bound on the integrator to prevent windup once ki_ is non-zero. Chosen so a
// saturated integral term alone can command full power: ki_ * INTEGRAL_MAX ~ 1.
static constexpr float INTEGRAL_MAX = 1000.0f;

static float clampf(float v, float lo, float hi) {
  return v < lo ? lo : (v > hi ? hi : v);
}

void PidController::reset() {
  integral_   = 0.0f;
  prev_error_ = 0.0f;
  has_prev_   = false;
}

float PidController::update(float temp_c, float setpoint_c) {
  // One temperature sample per call, so the loop time step is implicit (one
  // sample). dt cancels into the gains for now; make it explicit if the sample
  // cadence is changed.
  float error = setpoint_c - temp_c;
  float deriv = has_prev_ ? (error - prev_error_) : 0.0f;

  integral_   = clampf(integral_ + error, -INTEGRAL_MAX, INTEGRAL_MAX);
  prev_error_ = error;
  has_prev_   = true;

  float p   = kp_ * error;
  float i   = ki_ * integral_;
  float d   = kd_ * deriv;
  float out = clampf(p + i + d, 0.0f, 1.0f);

  terms_.error    = error;
  terms_.p        = p;
  terms_.i        = i;
  terms_.d        = d;
  terms_.integral = integral_;
  terms_.deriv    = deriv;
  terms_.output   = out;
  return out;
}

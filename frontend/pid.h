#pragma once

// Internal breakdown of the most recent update(), exposed for logging/tuning.
// All terms are from the same sample: output == clamp(p + i + d).
struct PidTerms {
  float error    = 0.0f; // residual, setpoint_c - temp_c
  float p        = 0.0f; // proportional contribution, kp_ * error
  float i        = 0.0f; // integral contribution, ki_ * integral
  float d        = 0.0f; // derivative contribution, kd_ * deriv
  float integral = 0.0f; // integrator state (pre-gain accumulator)
  float deriv    = 0.0f; // per-sample error delta (pre-gain)
  float output   = 0.0f; // commanded power after clamp, [0, 1]
};

// Temperature PID controller for the kettle.
//
// update() is run once per new temperature sample and returns a power command
// in [0, 1] that the caller maps onto the SSR duty cycles. The gains are
// currently zero, so the output is zero until the loop is tuned; the scaffolding
// (per-sample execution, output clamping, integral anti-windup, reset) is in
// place so enabling control is just a matter of setting the gains.
class PidController {
public:
  // Gains come from the ini file (see shared/config.h). They default to zero so
  // a default-constructed controller commands no power until tuned.
  PidController(float kp = 0.0f, float ki = 0.0f, float kd = 0.0f)
      : kp_(kp), ki_(ki), kd_(kd) {}

  // setpoint_c: target temperature. temp_c: latest filtered measurement.
  // Returns the commanded power in [0, 1].
  float update(float temp_c, float setpoint_c);

  // Clear the integrator and derivative history (e.g. on mode change).
  void reset();

  // Breakdown of the most recent update(), for logging and tuning.
  const PidTerms &terms() const { return terms_; }

private:
  float kp_;
  float ki_;
  float kd_;

  float integral_   = 0.0f;
  float prev_error_ = 0.0f;
  bool  has_prev_   = false;

  PidTerms terms_;
};

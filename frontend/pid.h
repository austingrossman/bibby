#pragma once

// Temperature PID controller for the kettle.
//
// update() is run once per new temperature sample and returns a power command
// in [0, 1] that the caller maps onto the SSR duty cycles. The gains are
// currently zero, so the output is zero until the loop is tuned; the scaffolding
// (per-sample execution, output clamping, integral anti-windup, reset) is in
// place so enabling control is just a matter of setting the gains.
class PidController {
public:
  // setpoint_c: target temperature. temp_c: latest filtered measurement.
  // Returns the commanded power in [0, 1].
  float update(float temp_c, float setpoint_c);

  // Clear the integrator and derivative history (e.g. on mode change).
  void reset();

private:
  float kp_ = 0.0f;
  float ki_ = 0.0f;
  float kd_ = 0.0f;

  float integral_   = 0.0f;
  float prev_error_ = 0.0f;
  bool  has_prev_   = false;
};

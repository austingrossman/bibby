#pragma once

// Online thermal-mass estimator. During any stretch where the delivered power
// is roughly constant and the (filtered) temperature rises by at least 1 degC,
// the kettle behaves as an integrator: dT/dt = P / (m*c). A least-squares
// slope over that stretch gives m*c = P / slope in J/degC — which is the batch
// size (water: ~4.186 kJ/degC per liter). The estimate is always computed and
// logged; using it to scale controller gains is a separate, config-gated
// decision made by the caller.
//
// The estimate ignores heat loss, which makes it read slightly high when the
// kettle is far above ambient; the caller's clamp bounds absorb that.

#define MC_EST_CAP 256

typedef struct {
  double t;
  float  temp;
  float  p;
} McSample;

typedef struct {
  float  min_power_w;    // ignore windows below this mean power
  McSample buf[MC_EST_CAP];  // decimated (t, T, P) ring over the last window
  int    head, count;
  double last_store_t;
  double last_eval_t;
  float  estimate;       // smoothed m*c, J/degC; 0 = no estimate yet
} McEstimator;

void mc_estimator_init(McEstimator *e, float min_power_w);

// Feed every control sample; decimation and evaluation pacing are internal.
void mc_estimator_push(McEstimator *e, double t_s, float temp_filt_c, float p_delivered_w);

// Smoothed m*c estimate in J/degC, or 0 while no valid window has been seen.
static inline float mc_estimator_value(const McEstimator *e) { return e->estimate; }

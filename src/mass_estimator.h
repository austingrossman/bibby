#pragma once

// Online batch-size estimator. During any stretch where the delivered power
// is roughly constant and the (filtered) temperature rises by at least 1 degC,
// the kettle behaves as an integrator: dT/dt = P / (m*c). A least-squares
// slope over that stretch gives the thermal mass P / slope in J/degC, which is
// reported as the equivalent litres of water, m = (P / slope) / c_water, so
// the readout is a batch size a brewer can check at a glance. The estimate
// is always computed and logged; using it to scale controller gains is a
// separate, config-gated decision made by the caller.
//
// The estimate ignores heat loss, which makes it read slightly high when the
// kettle is far above ambient, and it counts the kettle, elements and hoses
// as water. It also scales with the ratio of real to rated element watts: if
// the elements deliver 85 % of their rated power the estimate reads 1/0.85
// high. Compare it against the water actually put in to spot that.

// Specific heat of water, J per kg per degC; one litre of water is one kg.
#define WATER_C_J_PER_KG_C 4186.0f

#define MASS_EST_CAP 256

typedef struct {
  double t;
  float  temp;
  float  p;
} MassSample;

typedef struct {
  float  min_power_w;    // ignore windows below this mean power
  MassSample buf[MASS_EST_CAP];  // decimated (t, T, P) ring over the last window
  int    head, count;
  double last_store_t;
  double last_eval_t;
  float  estimate_l;     // smoothed batch size, litres of water; 0 = none yet
} MassEstimator;

void mass_estimator_init(MassEstimator *e, float min_power_w);

// Feed every control sample; decimation and evaluation pacing are internal.
void mass_estimator_push(MassEstimator *e, double t_s, float temp_filt_c, float p_delivered_w);

// Smoothed batch size in litres of water, or 0 while no valid window has been seen.
static inline float mass_estimator_value(const MassEstimator *e) { return e->estimate_l; }

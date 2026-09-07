#pragma once

// Split a total power demand across the two heating elements so that the
// surface power density (W/cm^2) is equal on both — the flux-minimizing split
// that spreads the load over all available element area (anti-scorch). Duty
// fraction appears here for the first time: it is the actuator command for the
// sigma-delta modulator, not a control quantity.

typedef struct {
  float p1_watts, p2_watts;     // rated element powers
  float a1_cm2,   a2_cm2;       // wetted element areas
  float max_flux_w_cm2;         // anti-scorch cap; 0 disables
} PowerSplitConfig;

typedef struct {
  float x1_w, x2_w;             // per-element power after split and clamping
  float duty1, duty2;           // per-element duty commands [0,1]
  float flux1_w_cm2, flux2_w_cm2;
} PowerSplitResult;

// Largest total power the split will deliver: P1 + P2, further capped by
// max_flux * (A1 + A2) when the anti-scorch cap is enabled.
float power_split_max_w(const PowerSplitConfig *cfg);

// Distribute p_demand_w (clamped to [0, power_split_max_w]) at equal flux,
// pushing overflow to the other element when one saturates at its rating.
PowerSplitResult power_split(const PowerSplitConfig *cfg, float p_demand_w);

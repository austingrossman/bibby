#include "power_split.h"

static float clampf(float v, float lo, float hi) {
  return v < lo ? lo : (v > hi ? hi : v);
}

float power_split_max_w(const PowerSplitConfig *cfg) {
  float max_w = cfg->p1_watts + cfg->p2_watts;
  if (cfg->max_flux_w_cm2 > 0.0f) {
    float flux_cap = cfg->max_flux_w_cm2 * (cfg->a1_cm2 + cfg->a2_cm2);
    if (flux_cap < max_w) max_w = flux_cap;
  }
  return max_w;
}

PowerSplitResult power_split(const PowerSplitConfig *cfg, float p_demand_w) {
  PowerSplitResult r = {0};
  float area = cfg->a1_cm2 + cfg->a2_cm2;
  if (area <= 0.0f || (cfg->p1_watts <= 0.0f && cfg->p2_watts <= 0.0f))
    return r;

  float p = clampf(p_demand_w, 0.0f, power_split_max_w(cfg));

  // Equal flux: x_i proportional to area. Minimizing max(x1/A1, x2/A2)
  // subject to x1 + x2 = p puts both fluxes equal.
  float x1 = p * cfg->a1_cm2 / area;
  float x2 = p * cfg->a2_cm2 / area;

  // One element saturating at its rating pushes the overflow to the other.
  if (x1 > cfg->p1_watts) { x2 += x1 - cfg->p1_watts; x1 = cfg->p1_watts; }
  if (x2 > cfg->p2_watts) { x1 += x2 - cfg->p2_watts; x2 = cfg->p2_watts; }
  x1 = clampf(x1, 0.0f, cfg->p1_watts);
  x2 = clampf(x2, 0.0f, cfg->p2_watts);

  r.x1_w  = x1;
  r.x2_w  = x2;
  r.duty1 = cfg->p1_watts > 0.0f ? x1 / cfg->p1_watts : 0.0f;
  r.duty2 = cfg->p2_watts > 0.0f ? x2 / cfg->p2_watts : 0.0f;
  r.flux1_w_cm2 = cfg->a1_cm2 > 0.0f ? x1 / cfg->a1_cm2 : 0.0f;
  r.flux2_w_cm2 = cfg->a2_cm2 > 0.0f ? x2 / cfg->a2_cm2 : 0.0f;
  return r;
}

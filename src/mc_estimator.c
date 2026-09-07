#include "mc_estimator.h"

#include <math.h>
#include <string.h>

// Window and pacing constants. 120 s of data at one stored point per 0.5 s;
// a fresh evaluation at most every 10 s. A window is only trusted when the
// power is steady (low relative spread) and the temperature moved enough to
// dominate sensor noise (>= 1 degC).
#define MC_WINDOW_S     120.0
#define MC_STORE_DT_S   0.5
#define MC_EVAL_DT_S    10.0
#define MC_MIN_RISE_C   1.0f
#define MC_MAX_P_SPREAD 0.05f   // max std(P)/mean(P) for "constant power"

void mc_estimator_init(McEstimator *e, float min_power_w) {
  memset(e, 0, sizeof(*e));
  e->min_power_w  = min_power_w;
  e->last_store_t = -1e18;
  e->last_eval_t  = -1e18;
}

// Least-squares slope of temp vs t plus power statistics over the ring.
static void window_stats(const McEstimator *e, double *slope_c_per_s,
                         float *p_mean, float *p_std, float *t_rise,
                         double *t_span) {
  int oldest = (e->head - e->count + MC_EST_CAP) % MC_EST_CAP;
  double t0 = e->buf[oldest].t;

  double sx = 0, sy = 0, sxx = 0, sxy = 0, sp = 0, spp = 0;
  float temp_min = 0, temp_max = 0;
  for (int i = 0; i < e->count; i++) {
    const McSample *s = &e->buf[(oldest + i) % MC_EST_CAP];
    double x = s->t - t0;
    double y = s->temp;
    sx  += x;   sy  += y;
    sxx += x*x; sxy += x*y;
    sp  += s->p; spp += (double)s->p * s->p;
    if (i == 0) { temp_min = temp_max = s->temp; }
    if (s->temp < temp_min) temp_min = s->temp;
    if (s->temp > temp_max) temp_max = s->temp;
  }
  double n     = (double)e->count;
  double denom = n * sxx - sx * sx;
  *slope_c_per_s = denom > 0 ? (n * sxy - sx * sy) / denom : 0.0;
  *p_mean = (float)(sp / n);
  double var = spp / n - (sp / n) * (sp / n);
  *p_std  = (float)(var > 0 ? sqrt(var) : 0.0);
  *t_rise = temp_max - temp_min;
  int newest = (e->head - 1 + MC_EST_CAP) % MC_EST_CAP;
  *t_span = e->buf[newest].t - t0;
}

void mc_estimator_push(McEstimator *e, double t_s, float temp_filt_c,
                       float p_delivered_w) {
  if (t_s - e->last_store_t < MC_STORE_DT_S) return;
  e->last_store_t = t_s;

  e->buf[e->head].t    = t_s;
  e->buf[e->head].temp = temp_filt_c;
  e->buf[e->head].p    = p_delivered_w;
  e->head = (e->head + 1) % MC_EST_CAP;
  if (e->count < MC_EST_CAP) e->count++;

  // Keep the ring trimmed to the analysis window.
  int oldest = (e->head - e->count + MC_EST_CAP) % MC_EST_CAP;
  while (e->count > 2 && t_s - e->buf[oldest].t > MC_WINDOW_S) {
    e->count--;
    oldest = (oldest + 1) % MC_EST_CAP;
  }

  if (t_s - e->last_eval_t < MC_EVAL_DT_S || e->count < 20) return;
  e->last_eval_t = t_s;

  double slope, t_span;
  float p_mean, p_std, t_rise;
  window_stats(e, &slope, &p_mean, &p_std, &t_rise, &t_span);

  if (t_span < MC_WINDOW_S * 0.9) return;               // window not full yet
  if (p_mean < e->min_power_w) return;                  // too little power
  if (p_std > MC_MAX_P_SPREAD * p_mean) return;         // power not constant
  if (t_rise < MC_MIN_RISE_C || slope <= 0.0) return;   // rise too small

  float mc = (float)(p_mean / slope);
  // Smooth accepted estimates so one window cannot yank the gain schedule.
  e->estimate = (e->estimate > 0.0f) ? 0.7f * e->estimate + 0.3f * mc : mc;
}

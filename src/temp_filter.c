#include "temp_filter.h"

#include <stdlib.h>

int temp_filter_init(TempFilter *f, int order, int window) {
  if (order < 1) order = 1;
  if (order > TEMP_FILTER_MAX_ORDER) order = TEMP_FILTER_MAX_ORDER;
  if (window < 1) window = 1;

  f->order  = order;
  f->window = window;
  f->primed = 0;
  f->value  = 0.0f;
  f->buf    = calloc((size_t)order * (size_t)window, sizeof(float));
  for (int s = 0; s < TEMP_FILTER_MAX_ORDER; s++) {
    f->sum[s] = 0.0f;
    f->idx[s] = 0;
  }
  return f->buf ? 0 : -1;
}

void temp_filter_free(TempFilter *f) {
  free(f->buf);
  f->buf = NULL;
}

// Advance one boxcar stage: drop the oldest sample, add the new one, return mean.
static float run_stage(TempFilter *f, int stage, float in) {
  float *buf = f->buf + (size_t)stage * (size_t)f->window;
  f->sum[stage] -= buf[f->idx[stage]];
  buf[f->idx[stage]] = in;
  f->sum[stage] += in;
  f->idx[stage] = (f->idx[stage] + 1) % f->window;
  return f->sum[stage] / (float)f->window;
}

float temp_filter_push(TempFilter *f, float sample) {
  if (!f->primed) {
    for (int s = 0; s < f->order; s++) {
      float *buf = f->buf + (size_t)s * (size_t)f->window;
      for (int i = 0; i < f->window; i++) buf[i] = sample;
      f->sum[s] = sample * (float)f->window;
    }
    f->primed = 1;
    return f->value = sample;
  }
  float v = sample;
  for (int s = 0; s < f->order; s++) v = run_stage(f, s, v);
  return f->value = v;
}

float temp_filter_group_delay(const TempFilter *f) {
  return (float)f->order * (float)(f->window - 1) * 0.5f;
}

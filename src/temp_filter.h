#pragma once

// Cascaded boxcar (moving-average) temperature filter — `order` identical
// N-sample stages in series, equivalent to an order-K CIC decimator running at
// one output per input. Order 2 gives triangular weighting (the proven
// default). The first sample primes every stage so the output starts at the
// true value instead of ramping from zero.
//
// Group delay is order * (window - 1) / 2 samples; at the sensor's ~50/60 Hz
// conversion rate this feeds the dead-time term used for tuning.

#define TEMP_FILTER_MAX_ORDER 4

typedef struct {
  int    order;    // stages, 1..TEMP_FILTER_MAX_ORDER
  int    window;   // samples per stage
  float *buf;      // order stages of window samples
  float  sum[TEMP_FILTER_MAX_ORDER];
  int    idx[TEMP_FILTER_MAX_ORDER];
  int    primed;
  float  value;
} TempFilter;

// Allocates the stage buffers; order/window are clamped to sane ranges.
// Returns 0 on success, -1 on allocation failure.
int temp_filter_init(TempFilter *f, int order, int window);

void temp_filter_free(TempFilter *f);

// Feed one sample; returns the new filtered value.
float temp_filter_push(TempFilter *f, float sample);

// Group delay in samples (for dead-time bookkeeping).
float temp_filter_group_delay(const TempFilter *f);

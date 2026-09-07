#pragma once

#include "config.h"
#include "csv_logger.h"
#include "max31865.h"
#include "state.h"

struct gpiod_line_request;

// Thread B: the RTD sampler and control loop. Blocks on the MAX31865 DRDY
// falling edge (fresh conversion every mains-notch period, ~50/60 Hz) and on
// each fresh sample runs: fault check -> Callendar-Van Dusen + calibration ->
// filter -> control law (degC -> W) -> power split (W -> duties) -> publish ->
// m*c estimator -> CSV log. Its heartbeat feeds the SSR thread's staleness
// watchdog, so it bumps every pass even when the sensor is dead — manual
// control must survive sensor loss.
typedef struct {
  BibbyState        *st;
  const BibbyConfig *cfg;
  Max31865          *sensor;    // NULL when sensor init failed (bench mode)
  CsvLogger         *logger;
  struct gpiod_line_request *drdy_req;  // GPIO16 input, falling-edge events
} SamplerThreadArgs;

void *sampler_thread_main(void *arg);

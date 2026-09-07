#pragma once

#include "config.h"
#include "state.h"

struct gpiod_line_request;

// Thread A: the SSR modulator. Runs at real-time priority off the mains
// zero-crossing edge; everything safety-critical about the outputs lives here.
//
// Per zero crossing (real, or simulated on timeout when simulate_zc is set):
//   - control-staleness watchdog: duties forced to 0 if the sampler heartbeat
//     has not advanced for ~2 s worth of crossings
//   - sigma-delta per element -> fire/skip this half-cycle
// On a timeout without simulate_zc: SSRs off, watchdog_alarm raised.
// On thread exit: SSRs driven low.
typedef struct {
  BibbyState        *st;
  const BibbyConfig *cfg;
  struct gpiod_line_request *zc_req;   // GPIO20 input, rising-edge events
  struct gpiod_line_request *ssr_req;  // GPIO21 + GPIO26 outputs
  unsigned ssr1_gpio, ssr2_gpio;
} SsrThreadArgs;

void *ssr_thread_main(void *arg);

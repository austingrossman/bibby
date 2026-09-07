#pragma once
#include <stdbool.h>

// First-order sigma-delta modulator clocked by the mains zero crossings.
// Each ZC, the commanded duty fraction is added to an accumulator; the SSR
// fires for that half-cycle when the accumulator reaches 1.0. The average
// fired fraction equals the commanded duty, with switching energy pushed to
// the highest frequency the SSR can express (whole half-cycles).
typedef struct {
  float acc;
} SigmaDelta;

// Advance one zero crossing with duty in [0,1]; returns true to fire the SSR
// for this half-cycle. The accumulator is capped at 2.0 to bound catch-up
// after a stretch of forced-off half-cycles.
static inline bool sigma_delta_step(SigmaDelta *sd, float duty) {
  sd->acc += duty;
  bool fire = (sd->acc >= 1.0f);
  if (fire) sd->acc -= 1.0f;
  if (sd->acc > 2.0f) sd->acc = 2.0f;
  return fire;
}

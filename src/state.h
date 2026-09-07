#pragma once
#include <pthread.h>
#include <stdatomic.h>
#include <stdbool.h>
#include <stdint.h>
#include <time.h>

// Shared in-process state between the three threads (SSR modulator, RTD
// sampler, GUI). Scalars are C11 atomics — each is an independent value and
// readers tolerate one-sample skew, so no locking is needed. The only compound
// shared object is the chart history ring, guarded by one mutex.

// One decimated control-loop snapshot for the on-screen chart.
typedef struct {
  float t_s;            // monotonic time, seconds
  float setpoint_c;
  float temp_filt_c;
  float temp_raw_c;
  float p_demand_w;     // control-law output (before split)
  float ff_w, p_w, i_w, d_w; // PID term breakdown, watts
  float duty1, duty2;
  float p_delivered_w;  // measured from fired half-cycles
  bool  manual;
  bool  grain_in;
  bool  fault;          // any RTD fault (register bits or unresponsive)
  bool  watchdog;       // ZC-lost alarm
} HistPoint;

#define HISTORY_CAP 4096  // at ~2 Hz push rate: > 30 min of chart history

typedef struct {
  // Process lifetime; cleared by the signal handler or any thread on fatal error.
  atomic_bool running;

  // Sampler -> SSR thread.
  _Atomic float   duty1;              // per-element duty command [0,1]
  _Atomic float   duty2;
  atomic_bool     simulate_zc;        // treat ZC-watchdog timeouts as crossings
  atomic_uint     control_heartbeat;  // bumped every sampler pass (staleness watchdog)

  // SSR thread -> others.
  atomic_bool     output1, output2;   // SSR fired state this half-cycle
  atomic_bool     watchdog_alarm;     // no ZC edge within the mains-derived timeout
  atomic_uint     zc_count;           // zero crossings seen (real or simulated)
  atomic_uint     fired1, fired2;     // cumulative fired half-cycles per element

  // Sampler -> UI.
  _Atomic float   temp_raw_c;
  _Atomic float   temp_filt_c;
  atomic_bool     temp_valid;         // at least one good conversion since start
  atomic_uchar    rtd_fault;          // MAX31865 fault register bits (0 = none)
  atomic_bool     rtd_unresponsive;   // DRDY silent — sensor missing/hung
  _Atomic float   p_demand_w;
  _Atomic float   p_delivered_w;      // rolling measured output power
  _Atomic float   mc_est_j_per_c;     // online thermal-mass estimate (0 = none yet)
  _Atomic float   adaptive_scale;     // gain scale in effect (1.0 when disabled)
  atomic_bool     fault_forced_manual;// auto was blocked/kicked by a sensor fault

  // UI -> sampler.
  _Atomic float   setpoint_c;
  atomic_bool     manual_mode;
  _Atomic float   manual_power_w;     // manual power command, watts
  atomic_bool     grain_in;

  // Chart history ring (sampler writes ~2 Hz, UI reads each frame).
  pthread_mutex_t hist_mutex;
  HistPoint       hist[HISTORY_CAP];
  int             hist_head;          // next write index
  int             hist_count;         // valid points, <= HISTORY_CAP
} BibbyState;

void state_init(BibbyState *st);

void state_history_push(BibbyState *st, const HistPoint *pt);

// Copy the points newer than t_min_s (oldest first) into out; returns count.
int state_history_snapshot(BibbyState *st, HistPoint *out, int max, double t_min_s);

// Monotonic clock in seconds — the shared time base for control and logging.
static inline double bibby_now_s(void) {
  struct timespec ts;
  clock_gettime(CLOCK_MONOTONIC, &ts);
  return (double)ts.tv_sec + (double)ts.tv_nsec * 1e-9;
}

#include "sampler_thread.h"

#include <gpiod.h>
#include <stdio.h>
#include <stdlib.h>

#include "control.h"
#include "mass_estimator.h"
#include "power_split.h"
#include "temp_filter.h"

// DRDY edge wait timeout. Also the loop pace with no sensor: manual commands
// and the heartbeat are still serviced at this rate.
#define DRDY_TIMEOUT_NS   500000000LL   // 500 ms
// No fresh conversion for this long (with a sensor present) = sensor fault.
#define UNRESPONSIVE_S    1.0
// Cadence for the delivered-power measurement and the chart history push.
#define SLOW_TICK_S       0.5

static float clampf(float v, float lo, float hi) {
  return v < lo ? lo : (v > hi ? hi : v);
}

// Measured output power over the last slow tick, from the SSR thread's fired
// half-cycle counters: delivered = (fired fraction) x (rated watts), summed.
typedef struct {
  uint32_t zc, fired1, fired2;
} FiredSnapshot;

static float delivered_power_w(BibbyState *st, const BibbyConfig *cfg,
                               FiredSnapshot *prev) {
  FiredSnapshot cur = {
    atomic_load(&st->zc_count),
    atomic_load(&st->fired1),
    atomic_load(&st->fired2),
  };
  uint32_t dzc = cur.zc - prev->zc;
  float p = 0.0f;
  if (dzc > 0) {
    p = ((float)(cur.fired1 - prev->fired1) * cfg->element1_watts +
         (float)(cur.fired2 - prev->fired2) * cfg->element2_watts) / (float)dzc;
  }
  *prev = cur;
  return p;
}

void *sampler_thread_main(void *arg) {
  SamplerThreadArgs *a  = arg;
  BibbyState        *st = a->st;
  const BibbyConfig *cfg = a->cfg;

  const PowerSplitConfig split_cfg = {
    cfg->element1_watts, cfg->element2_watts,
    cfg->element1_area_cm2, cfg->element2_area_cm2,
    cfg->max_flux_w_cm2,
  };
  const float split_max_w = power_split_max_w(&split_cfg);

  TempFilter filter;
  if (temp_filter_init(&filter, cfg->filter_order, cfg->filter_window) != 0) {
    fprintf(stderr, "sampler: filter allocation failed\n");
    atomic_store(&st->running, false);
    return NULL;
  }

  Pid pid;
  pid_init(&pid, cfg->pid_kp, cfg->pid_ki, cfg->pid_kd);
  pid_set_i_band(&pid, cfg->pid_i_band_c);

  MassEstimator mass;
  // Ignore rise windows below 10% of rated power — too little signal.
  mass_estimator_init(&mass, 0.10f * config_total_watts(cfg));

  struct gpiod_edge_event_buffer *event_buf = gpiod_edge_event_buffer_new(4);
  if (!event_buf) {
    perror("sampler: edge event buffer");
    temp_filter_free(&filter);
    atomic_store(&st->running, false);
    return NULL;
  }

  FiredSnapshot fired_prev = {0};
  double last_fresh_t  = bibby_now_s();
  // Start the slow-tick clock now: firing it on the first pass would push a
  // history point before the first fresh sample (temp reads as 0 °C there,
  // which pins the chart auto-scale to zero for the whole window).
  double last_slow_t   = bibby_now_s();
  double last_log_t    = 0.0;
  float  temp_filt     = 0.0f;
  float  p_delivered   = 0.0f;
  bool   prev_manual   = true;
  bool   prev_grain    = false;

  // Bench hook for the bring-up test plan (README): BIBBY_TEST_WEDGE=<sec>
  // wedges this thread once, 5 s in, so the SSR thread's control-staleness
  // watchdog can be watched zeroing the duties. Unset in normal operation.
  double wedge_s  = getenv("BIBBY_TEST_WEDGE") ? atof(getenv("BIBBY_TEST_WEDGE")) : 0.0;
  double wedge_at = bibby_now_s() + 5.0;

  while (atomic_load(&st->running)) {
    // Every pass bumps the heartbeat: the staleness watchdog must only trip
    // when this loop is actually wedged, not when the sensor is quiet.
    atomic_fetch_add(&st->control_heartbeat, 1);

    if (wedge_s > 0.0 && bibby_now_s() >= wedge_at) {
      fprintf(stderr, "sampler: TEST WEDGE for %.1f s\n", wedge_s);
      struct timespec ts = { (time_t)wedge_s, 0 };
      nanosleep(&ts, NULL);
      wedge_s = 0.0;
    }

    bool fresh = false;
    float temp_raw = 0.0f;

    if (a->sensor) {
      int ret = gpiod_line_request_wait_edge_events(a->drdy_req, DRDY_TIMEOUT_NS);
      if (ret > 0)
        gpiod_line_request_read_edge_events(a->drdy_req, event_buf, 4);
      // Read regardless of the wait result: max31865_read checks the DRDY
      // level itself, so a missed edge (or one pending before the request was
      // armed) still yields the conversion instead of stalling forever.
      temp_raw = max31865_read(a->sensor, &fresh);
      atomic_store(&st->rtd_fault, max31865_fault(a->sensor));
    } else {
      struct timespec ts = { 0, 100000000L };  // no sensor: idle at 10 Hz
      nanosleep(&ts, NULL);
    }

    double now = bibby_now_s();

    if (fresh) {
      last_fresh_t = now;
      atomic_store(&st->rtd_unresponsive, false);
      temp_filt = temp_filter_push(&filter, temp_raw);
      atomic_store(&st->temp_raw_c, temp_raw);
      atomic_store(&st->temp_filt_c, temp_filt);
      atomic_store(&st->temp_valid, true);
    } else if (now - last_fresh_t > UNRESPONSIVE_S) {
      atomic_store(&st->rtd_unresponsive, true);
    }

    // ── Mode interlock ───────────────────────────────────────────────────
    bool fault = atomic_load(&st->rtd_fault) != 0 ||
                 atomic_load(&st->rtd_unresponsive) ||
                 !atomic_load(&st->temp_valid);
    bool manual = atomic_load(&st->manual_mode);
    if (fault && !manual) {
      // Unreliable temperature: automatic control may not command power.
      // Enter manual at zero watts and say why.
      atomic_store(&st->manual_power_w, 0.0f);
      atomic_store(&st->manual_mode, true);
      atomic_store(&st->fault_forced_manual, true);
      manual = true;
    }
    if (!fault) atomic_store(&st->fault_forced_manual, false);
    if (manual != prev_manual) {
      pid_reset(&pid);          // no stale integrator across a mode change
      prev_manual = manual;
    }

    // Grain-in swaps the gain set and (optionally) lowers peak power.
    bool grain = atomic_load(&st->grain_in);
    if (grain != prev_grain) {
      if (grain) pid_set_gains(&pid, cfg->grain_kp, cfg->grain_ki, cfg->grain_kd);
      else       pid_set_gains(&pid, cfg->pid_kp, cfg->pid_ki, cfg->pid_kd);
      prev_grain = grain;
    }
    float out_max_w = split_max_w;
    if (grain && cfg->grain_max_power_w > 0.0f && cfg->grain_max_power_w < out_max_w)
      out_max_w = cfg->grain_max_power_w;

    // Adaptive batch-size gain schedule (config-gated; estimate always runs).
    float scale = 1.0f;
    float m_est = mass_estimator_value(&mass);
    if (cfg->adaptive_enable && cfg->adaptive_m_ref_l > 0.0f && m_est > 0.0f)
      scale = clampf(m_est / cfg->adaptive_m_ref_l,
                     cfg->adaptive_scale_min, cfg->adaptive_scale_max);
    atomic_store(&st->adaptive_scale, scale);

    // ── Command path ─────────────────────────────────────────────────────
    // Manual publishes every pass (a dead sensor must not freeze the slider);
    // auto runs the control law once per fresh sample.
    bool publish = false;
    float p_demand = atomic_load(&st->p_demand_w);
    if (manual) {
      p_demand = clampf(atomic_load(&st->manual_power_w), 0.0f, out_max_w);
      pid.terms = (PidTerms){0};
      pid.terms.error_c  = atomic_load(&st->setpoint_c) - temp_filt;
      pid.terms.output_w = p_demand;
      publish = true;
    } else if (fresh) {
      float setpoint = atomic_load(&st->setpoint_c);
      float ff_w = control_feedforward_w(setpoint, cfg->ff_ambient_c,
                                         cfg->ff_process_gain_c, out_max_w);
      p_demand = pid_update(&pid, temp_filt, setpoint, ff_w, out_max_w, scale);
      publish = true;
    }

    PowerSplitResult split = power_split(&split_cfg, p_demand);
    if (publish) {
      atomic_store(&st->p_demand_w, p_demand);
      atomic_store(&st->duty1, split.duty1);
      atomic_store(&st->duty2, split.duty2);
    }

    // ── Slow tick: delivered power, batch-size estimate, chart history ───
    if (now - last_slow_t >= SLOW_TICK_S) {
      last_slow_t = now;
      p_delivered = delivered_power_w(st, cfg, &fired_prev);
      atomic_store(&st->p_delivered_w, p_delivered);
      atomic_store(&st->m_est_l, m_est);

      HistPoint pt = {
        .t_s         = (float)now,
        .setpoint_c  = atomic_load(&st->setpoint_c),
        .temp_filt_c = temp_filt,
        .temp_raw_c  = atomic_load(&st->temp_raw_c),
        .p_demand_w  = p_demand,
        .ff_w = pid.terms.ff_w, .p_w = pid.terms.p_w,
        .i_w = pid.terms.i_w,   .d_w = pid.terms.d_w,
        .duty1 = split.duty1, .duty2 = split.duty2,
        .p_delivered_w = p_delivered,
        .manual   = manual,
        .grain_in = grain,
        .fault    = fault,
        .watchdog = atomic_load(&st->watchdog_alarm),
      };
      state_history_push(st, &pt);
    }

    // One row per fresh sample; during a fault (no fresh data) fall back to
    // one row per slow tick so the outage and its fault bits stay on disk.
    bool log_now = fresh || (now - last_fresh_t > SLOW_TICK_S &&
                             now - last_log_t >= SLOW_TICK_S);
    if (fresh) mass_estimator_push(&mass, now, temp_filt, p_delivered);
    if (log_now) {
      last_log_t = now;

      LogRow row = {
        .t_monotonic_s = now,
        .temp_raw_c    = fresh ? temp_raw : atomic_load(&st->temp_raw_c),
        .temp_filt_c   = temp_filt,
        .setpoint_c    = atomic_load(&st->setpoint_c),
        .p_demand_w    = p_demand,
        .p_delivered_w = p_delivered,
        .duty1 = split.duty1, .duty2 = split.duty2,
        .flux1_w_cm2 = split.flux1_w_cm2, .flux2_w_cm2 = split.flux2_w_cm2,
        .pid = pid.terms,
        .m_est_l   = m_est,
        .manual    = manual,
        .grain_in  = grain,
        .rtd_fault = atomic_load(&st->rtd_fault),
        .watchdog  = atomic_load(&st->watchdog_alarm),
      };
      csv_logger_log(a->logger, &row);
    }
  }

  gpiod_edge_event_buffer_free(event_buf);
  temp_filter_free(&filter);
  atomic_store(&st->running, false);
  return NULL;
}

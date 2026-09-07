#include "ssr_thread.h"

#include <gpiod.h>
#include <pthread.h>
#include <sched.h>
#include <stdio.h>

#include "sigma_delta.h"

static float clamp01(float v) {
  return v < 0.0f ? 0.0f : (v > 1.0f ? 1.0f : v);
}

static void ssrs_off(const SsrThreadArgs *a) {
  gpiod_line_request_set_value(a->ssr_req, a->ssr1_gpio, GPIOD_LINE_VALUE_INACTIVE);
  gpiod_line_request_set_value(a->ssr_req, a->ssr2_gpio, GPIOD_LINE_VALUE_INACTIVE);
  atomic_store(&a->st->output1, false);
  atomic_store(&a->st->output2, false);
}

// SCHED_FIFO at modest priority so GUI or GL stalls can never delay the
// zero-cross response or the watchdogs. Failure (no CAP_SYS_NICE) is reported
// but not fatal — the loop still works, just without the RT guarantee.
static void try_realtime(void) {
  struct sched_param sp = { .sched_priority = 20 };
  if (pthread_setschedparam(pthread_self(), SCHED_FIFO, &sp) != 0)
    fprintf(stderr, "ssr_thread: SCHED_FIFO unavailable (run with "
                    "CAP_SYS_NICE for RT scheduling); continuing\n");
}

void *ssr_thread_main(void *arg) {
  SsrThreadArgs *a  = arg;
  BibbyState    *st = a->st;

  try_realtime();

  const uint64_t zc_timeout_ns = config_zc_timeout_ns(a->cfg);
  const uint32_t staleness_zc  = config_staleness_zc(a->cfg);

  struct gpiod_edge_event_buffer *event_buf = gpiod_edge_event_buffer_new(1);
  if (!event_buf) {
    perror("ssr_thread: edge event buffer");
    atomic_store(&st->running, false);
    return NULL;
  }

  SigmaDelta sd1 = {0}, sd2 = {0};
  uint32_t last_heartbeat = 0;
  uint32_t stale_zc       = 0;
  bool     control_seen   = false;
  bool     was_stale      = false;

  while (atomic_load(&st->running)) {
    int ret = gpiod_line_request_wait_edge_events(a->zc_req, (int64_t)zc_timeout_ns);
    if (ret < 0) {
      perror("ssr_thread: wait_edge_events");
      break;
    }

    if (ret == 0) {
      // No zero crossing within one half-period + guard. With simulation on
      // this timeout IS the (fake) crossing; otherwise mains sense is lost:
      // outputs off, alarm up, keep waiting.
      if (!atomic_load(&st->simulate_zc)) {
        atomic_store(&st->watchdog_alarm, true);
        ssrs_off(a);
        continue;
      }
    } else {
      if (gpiod_line_request_read_edge_events(a->zc_req, event_buf, 1) < 1)
        continue;
      struct gpiod_edge_event *ev = gpiod_edge_event_buffer_get_event(event_buf, 0);
      if (gpiod_edge_event_get_event_type(ev) != GPIOD_EDGE_EVENT_RISING_EDGE)
        continue;
    }

    atomic_fetch_add(&st->zc_count, 1);
    atomic_store(&st->watchdog_alarm, false);

    float d1 = clamp01(atomic_load(&st->duty1));
    float d2 = clamp01(atomic_load(&st->duty2));

    // Control-staleness watchdog: the sampler bumps its heartbeat every pass;
    // if it stops (thread wedged, sensor loop dead), stop delivering power.
    uint32_t hb = atomic_load(&st->control_heartbeat);
    if (hb != last_heartbeat) {
      last_heartbeat = hb;
      stale_zc       = 0;
      control_seen   = true;
    } else if (control_seen) {
      stale_zc++;
    }
    bool stale = !control_seen || stale_zc >= staleness_zc;
    if (stale) {
      d1 = 0.0f;
      d2 = 0.0f;
    }
    if (stale != was_stale) {
      fprintf(stderr, stale ? "ssr_thread: control stale -> duties forced 0\n"
                            : "ssr_thread: control heartbeat recovered\n");
      was_stale = stale;
    }

    bool fire1 = sigma_delta_step(&sd1, d1);
    bool fire2 = sigma_delta_step(&sd2, d2);

    gpiod_line_request_set_value(a->ssr_req, a->ssr1_gpio,
        fire1 ? GPIOD_LINE_VALUE_ACTIVE : GPIOD_LINE_VALUE_INACTIVE);
    gpiod_line_request_set_value(a->ssr_req, a->ssr2_gpio,
        fire2 ? GPIOD_LINE_VALUE_ACTIVE : GPIOD_LINE_VALUE_INACTIVE);
    atomic_store(&st->output1, fire1);
    atomic_store(&st->output2, fire2);
    if (fire1) atomic_fetch_add(&st->fired1, 1);
    if (fire2) atomic_fetch_add(&st->fired2, 1);
  }

  ssrs_off(a);
  gpiod_edge_event_buffer_free(event_buf);
  atomic_store(&st->running, false);  // take the rest of the process down too
  return NULL;
}

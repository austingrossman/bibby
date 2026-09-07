#include "state.h"

#include <string.h>

void state_init(BibbyState *st) {
  memset(st, 0, sizeof(*st));
  atomic_store(&st->running, true);
  atomic_store(&st->manual_mode, true);   // manual until the user enables auto
  atomic_store(&st->setpoint_c, 65.0f);
  atomic_store(&st->adaptive_scale, 1.0f);
  pthread_mutex_init(&st->hist_mutex, NULL);
}

void state_history_push(BibbyState *st, const HistPoint *pt) {
  pthread_mutex_lock(&st->hist_mutex);
  st->hist[st->hist_head] = *pt;
  st->hist_head = (st->hist_head + 1) % HISTORY_CAP;
  if (st->hist_count < HISTORY_CAP) st->hist_count++;
  pthread_mutex_unlock(&st->hist_mutex);
}

int state_history_snapshot(BibbyState *st, HistPoint *out, int max, double t_min_s) {
  pthread_mutex_lock(&st->hist_mutex);
  int n = 0;
  int oldest = (st->hist_head - st->hist_count + HISTORY_CAP) % HISTORY_CAP;
  for (int i = 0; i < st->hist_count && n < max; i++) {
    const HistPoint *pt = &st->hist[(oldest + i) % HISTORY_CAP];
    if (pt->t_s < t_min_s) continue;
    out[n++] = *pt;
  }
  pthread_mutex_unlock(&st->hist_mutex);
  return n;
}

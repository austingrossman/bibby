// Main bibby screen, 1280x720 logical (sideways 720x1280 panel).
//
//  +------------------------------+-------+--------------------+
//  | toggles      | set-pt steps  | POWER |  Set 65.0 C        |
//  |--------------+---------------| slider|   [ kettle ]       |
//  | temperature chart            |       |    65.32 C         |
//  |------------------------------|       |    149.6 F         |
//  | power / PID-term chart       |       |   (elements glow)  |
//  | fault band       | status    |       |  E1 50%   E2 50%   |
//  +------------------------------+-------+--------------------+
//
// The UI is a pure view/controller: it reads the shared state snapshot and
// writes setpoint / mode / manual watts / grain / sim-zc. Nothing here is in
// the control or safety path.
#include "ui_screen.h"

#include <stdint.h>
#include <stdio.h>

#include "../max31865.h"
#include "../power_split.h"
#include "lvgl.h"
#include "ui_kettle.h"
#include "ui_theme.h"

#define CHART_MAX_PTS 3600  // 30 min at one point per 0.5 s

static BibbyState        *ST;
static const BibbyConfig *CFG;

static struct {
  lv_obj_t *btn_manual, *btn_zc_sim, *btn_grain;
  lv_obj_t *slider, *slider_watts;
  lv_obj_t *lbl_setpoint, *lbl_temp_c, *lbl_temp_f, *lbl_elements;
  lv_obj_t *chart_temp, *chart_pow;
  lv_obj_t *ylab_temp[3], *ylab_pow[3];
  lv_obj_t *fault_band, *lbl_fault, *lbl_status;

  lv_chart_series_t *s_set, *s_filt, *s_raw, *s_grain_ev, *s_mode_ev;
  lv_chart_series_t *s_demand, *s_deliv, *s_ff, *s_p, *s_i, *s_d;

  int    chart_pts;
  float  max_watts;
  float  bright1, bright2;
  bool   was_manual;
} S;

// Chart ext arrays (LVGL reads these directly; we own the memory).
static int32_t a_set[CHART_MAX_PTS], a_filt[CHART_MAX_PTS], a_raw[CHART_MAX_PTS];
static int32_t a_grain_ev[CHART_MAX_PTS], a_mode_ev[CHART_MAX_PTS];
static int32_t a_demand[CHART_MAX_PTS], a_deliv[CHART_MAX_PTS], a_ff[CHART_MAX_PTS];
static int32_t a_p[CHART_MAX_PTS], a_i[CHART_MAX_PTS], a_d[CHART_MAX_PTS];
static HistPoint snap[CHART_MAX_PTS];

// Per-trace visibility, driven by the legend chips under each chart. A hidden
// trace is neither drawn nor counted in that chart's auto-range, so switching
// off the terms you are not watching zooms the pane onto the rest.
enum {
  V_SET, V_FILT, V_RAW, V_GRAIN, V_MODE,
  V_DEMAND, V_DELIV, V_FF, V_P, V_I, V_D, V_COUNT
};
static bool vis[V_COUNT];
static struct {
  lv_obj_t           *chart;
  lv_chart_series_t **ser;
} trace[V_COUNT];

static float clampf(float v, float lo, float hi) {
  return v < lo ? lo : (v > hi ? hi : v);
}

// ── Widget helpers ────────────────────────────────────────────────────────────

static lv_obj_t *make_panel(lv_obj_t *parent, int x, int y, int w, int h) {
  lv_obj_t *o = lv_obj_create(parent);
  lv_obj_set_pos(o, x, y);
  lv_obj_set_size(o, w, h);
  lv_obj_set_style_bg_color(o, lv_color_hex(UI_PANEL), 0);
  lv_obj_set_style_border_color(o, lv_color_hex(UI_BORDER), 0);
  lv_obj_set_style_border_width(o, 1, 0);
  lv_obj_set_style_radius(o, 8, 0);
  lv_obj_set_style_pad_all(o, 0, 0);
  lv_obj_remove_flag(o, LV_OBJ_FLAG_SCROLLABLE);
  return o;
}

static lv_obj_t *make_label(lv_obj_t *parent, int x, int y, const lv_font_t *font,
                            uint32_t color, const char *txt) {
  lv_obj_t *l = lv_label_create(parent);
  lv_obj_set_pos(l, x, y);
  lv_obj_set_style_text_font(l, font, 0);
  lv_obj_set_style_text_color(l, lv_color_hex(color), 0);
  lv_label_set_text(l, txt);
  return l;
}

// Big finger-sized toggle: grey when off, colored when on.
static lv_obj_t *make_toggle(lv_obj_t *parent, const char *txt, int x, int y,
                             int w, int h, uint32_t on_color,
                             lv_event_cb_t cb) {
  lv_obj_t *b = lv_button_create(parent);
  lv_obj_set_pos(b, x, y);
  lv_obj_set_size(b, w, h);
  lv_obj_add_flag(b, LV_OBJ_FLAG_CHECKABLE);
  lv_obj_set_style_bg_color(b, lv_color_hex(0x3a3a44), 0);
  lv_obj_set_style_bg_color(b, lv_color_hex(on_color), LV_STATE_CHECKED);
  lv_obj_set_style_radius(b, 10, 0);
  lv_obj_set_style_shadow_width(b, 0, 0);
  lv_obj_t *l = lv_label_create(b);
  lv_label_set_text(l, txt);
  lv_obj_set_style_text_font(l, &lv_font_montserrat_20, 0);
  lv_obj_center(l);
  if (cb) lv_obj_add_event_cb(b, cb, LV_EVENT_VALUE_CHANGED, NULL);
  return b;
}

// ── Control events ────────────────────────────────────────────────────────────

static void manual_toggle_cb(lv_event_t *e) {
  lv_obj_t *btn = lv_event_get_target(e);
  bool want_manual = lv_obj_has_state(btn, LV_STATE_CHECKED);
  if (want_manual) {
    // Bumpless takeover: the slider starts from what the loop was commanding.
    atomic_store(&ST->manual_power_w, atomic_load(&ST->p_demand_w));
    atomic_store(&ST->manual_mode, true);
  } else {
    // Auto is only reachable with a healthy sensor; the sampler enforces this
    // too, but reflect it immediately instead of flickering.
    bool fault = atomic_load(&ST->rtd_fault) != 0 ||
                 atomic_load(&ST->rtd_unresponsive) ||
                 !atomic_load(&ST->temp_valid);
    if (fault) {
      lv_obj_add_state(btn, LV_STATE_CHECKED);
      return;
    }
    atomic_store(&ST->manual_mode, false);
  }
}

static void zc_sim_toggle_cb(lv_event_t *e) {
  lv_obj_t *btn = lv_event_get_target(e);
  atomic_store(&ST->simulate_zc, lv_obj_has_state(btn, LV_STATE_CHECKED));
}

static void grain_toggle_cb(lv_event_t *e) {
  lv_obj_t *btn = lv_event_get_target(e);
  atomic_store(&ST->grain_in, lv_obj_has_state(btn, LV_STATE_CHECKED));
}

static void step_cb(lv_event_t *e) {
  float step = *(const float *)lv_event_get_user_data(e);
  float sp = clampf(atomic_load(&ST->setpoint_c) + step, 0.0f, 105.0f);
  atomic_store(&ST->setpoint_c, sp);
}

static void slider_cb(lv_event_t *e) {
  lv_obj_t *sl = lv_event_get_target(e);
  if (atomic_load(&ST->manual_mode))
    atomic_store(&ST->manual_power_w, (float)lv_slider_get_value(sl));
}

// ── Chart plumbing ────────────────────────────────────────────────────────────

static lv_chart_series_t *add_series(lv_obj_t *chart, uint32_t color,
                                     int32_t *array) {
  lv_chart_series_t *s =
      lv_chart_add_series(chart, lv_color_hex(color), LV_CHART_AXIS_PRIMARY_Y);
  lv_chart_set_series_ext_y_array(chart, s, array);
  return s;
}

// Legend chips: checked = trace visible. Colored when on, grey when off.
static void legend_cb(lv_event_t *e) {
  lv_obj_t *chip = lv_event_get_target(e);
  int       idx  = (int)(intptr_t)lv_event_get_user_data(e);
  vis[idx] = lv_obj_has_state(chip, LV_STATE_CHECKED);
  lv_chart_hide_series(trace[idx].chart, *trace[idx].ser, !vis[idx]);
}

static lv_obj_t *make_legend_row(lv_obj_t *parent, int x, int y, int w) {
  lv_obj_t *r = lv_obj_create(parent);
  lv_obj_set_pos(r, x, y);
  lv_obj_set_size(r, w, 30);
  lv_obj_set_style_bg_opa(r, LV_OPA_TRANSP, 0);
  lv_obj_set_style_border_width(r, 0, 0);
  lv_obj_set_style_pad_all(r, 0, 0);
  lv_obj_set_style_pad_column(r, 6, 0);
  lv_obj_remove_flag(r, LV_OBJ_FLAG_SCROLLABLE);
  lv_obj_set_flex_flow(r, LV_FLEX_FLOW_ROW);
  lv_obj_set_flex_align(r, LV_FLEX_ALIGN_START, LV_FLEX_ALIGN_CENTER,
                        LV_FLEX_ALIGN_CENTER);
  return r;
}

// One chip per trace: registers the series and starts visible. The chip is a
// full-height 30 px button so it is still a fair touch target on the panel.
static void make_chip(lv_obj_t *row, int idx, lv_obj_t *chart,
                      lv_chart_series_t **ser, uint32_t color,
                      const char *txt) {
  trace[idx].chart = chart;
  trace[idx].ser   = ser;
  vis[idx]         = true;

  lv_obj_t *b = lv_button_create(row);
  lv_obj_set_size(b, LV_SIZE_CONTENT, 30);
  lv_obj_set_style_min_width(b, 46, 0);
  lv_obj_set_style_pad_hor(b, 9, 0);
  lv_obj_set_style_radius(b, 6, 0);
  lv_obj_set_style_shadow_width(b, 0, 0);
  lv_obj_set_style_border_width(b, 1, 0);
  lv_obj_set_style_bg_color(b, lv_color_hex(UI_BG), 0);
  lv_obj_set_style_border_color(b, lv_color_hex(UI_BORDER), 0);
  lv_obj_set_style_text_color(b, lv_color_hex(0x5a6472), 0);
  // Struck through when off, so the two muted traces (raw, D) can't be
  // mistaken for disabled chips.
  lv_obj_set_style_text_decor(b, LV_TEXT_DECOR_STRIKETHROUGH, 0);
  lv_obj_set_style_text_decor(b, LV_TEXT_DECOR_NONE, LV_STATE_CHECKED);
  lv_obj_set_style_bg_color(b, lv_color_hex(UI_PANEL_HI), LV_STATE_CHECKED);
  lv_obj_set_style_border_color(b, lv_color_hex(color), LV_STATE_CHECKED);
  lv_obj_set_style_text_color(b, lv_color_hex(color), LV_STATE_CHECKED);
  lv_obj_add_flag(b, LV_OBJ_FLAG_CHECKABLE);
  lv_obj_add_state(b, LV_STATE_CHECKED);

  lv_obj_t *l = lv_label_create(b);
  lv_label_set_text(l, txt);
  lv_obj_set_style_text_font(l, &lv_font_montserrat_14, 0);
  lv_obj_center(l);
  lv_obj_add_event_cb(b, legend_cb, LV_EVENT_VALUE_CHANGED,
                      (void *)(intptr_t)idx);
}

static void fold(int32_t v, int32_t *lo, int32_t *hi) {
  if (v < *lo) *lo = v;
  if (v > *hi) *hi = v;
}

static lv_obj_t *make_chart(lv_obj_t *parent, int x, int y, int w, int h) {
  lv_obj_t *c = lv_chart_create(parent);
  lv_obj_set_pos(c, x, y);
  lv_obj_set_size(c, w, h);
  lv_chart_set_type(c, LV_CHART_TYPE_LINE);
  lv_chart_set_point_count(c, S.chart_pts);
  lv_chart_set_div_line_count(c, 5, 7);
  lv_obj_set_style_bg_color(c, lv_color_hex(UI_PANEL), 0);
  lv_obj_set_style_border_color(c, lv_color_hex(UI_BORDER), 0);
  lv_obj_set_style_line_color(c, lv_color_hex(0x262e38), LV_PART_MAIN);
  lv_obj_set_style_radius(c, 6, 0);
  lv_obj_set_style_pad_all(c, 2, 0);
  lv_obj_set_style_size(c, 0, 0, LV_PART_INDICATOR);  // no point dots
  lv_obj_set_style_line_width(c, 2, LV_PART_ITEMS);
  return c;
}

// Right-align history into the ext arrays by time (one slot per 0.5 s); slots
// with no sample stay LV_CHART_POINT_NONE so outages appear as gaps.
static void update_charts(void) {
  const int    pts = S.chart_pts;
  const double now = bibby_now_s();
  const double window_s = pts * 0.5;

  int n = state_history_snapshot(ST, snap, pts, now - window_s);

  for (int i = 0; i < pts; i++) {
    a_set[i] = a_filt[i] = a_raw[i] = LV_CHART_POINT_NONE;
    a_grain_ev[i] = a_mode_ev[i] = LV_CHART_POINT_NONE;
    a_demand[i] = a_deliv[i] = a_ff[i] = LV_CHART_POINT_NONE;
    a_p[i] = a_i[i] = a_d[i] = LV_CHART_POINT_NONE;
  }

  // Ranges while filling. Temperatures in centi-degC; powers in watts.
  int32_t tmin = 0, tmax = 0, pmin = 0, pmax = 0;
  bool t_have = false, p_have = false;
  int prev_i = -1;
  for (int k = 0; k < n; k++) {
    int i = pts - 1 - (int)((now - snap[k].t_s) / 0.5);
    if (i < 0 || i >= pts) continue;

    // The slow tick drifts slightly past its 0.5 s period, so consecutive
    // samples occasionally skip a slot; bridge holes of 1-2 slots so the
    // traces stay continuous. Longer holes are real outages and stay gaps.
    int from = (prev_i >= 0 && i - prev_i >= 2 && i - prev_i <= 3) ? prev_i + 1 : i;
    prev_i = i;
    for (int j = from; j <= i; j++) {
      a_set[j]    = (int32_t)(snap[k].setpoint_c * 100.0f);
      a_filt[j]   = (int32_t)(snap[k].temp_filt_c * 100.0f);
      a_raw[j]    = (int32_t)(snap[k].temp_raw_c * 100.0f);
      a_demand[j] = (int32_t)snap[k].p_demand_w;
      a_deliv[j]  = (int32_t)snap[k].p_delivered_w;
      a_ff[j]     = (int32_t)snap[k].ff_w;
      a_p[j]      = (int32_t)snap[k].p_w;
      a_i[j]      = (int32_t)snap[k].i_w;
      a_d[j]      = (int32_t)snap[k].d_w;
    }

    // Only the traces the legend leaves enabled drive the auto-range, so
    // hiding a trace zooms the pane onto whatever is still shown.
    int32_t tlo = INT32_MAX, thi = INT32_MIN;
    if (vis[V_SET])  fold(a_set[i],  &tlo, &thi);
    if (vis[V_FILT]) fold(a_filt[i], &tlo, &thi);
    if (vis[V_RAW])  fold(a_raw[i],  &tlo, &thi);
    if (tlo <= thi) {
      tmin = t_have ? LV_MIN(tmin, tlo) : tlo;
      tmax = t_have ? LV_MAX(tmax, thi) : thi;
      t_have = true;
    }

    int32_t plo = INT32_MAX, phi = INT32_MIN;
    if (vis[V_DEMAND]) fold(a_demand[i], &plo, &phi);
    if (vis[V_DELIV])  fold(a_deliv[i],  &plo, &phi);
    if (vis[V_FF])     fold(a_ff[i],     &plo, &phi);
    if (vis[V_P])      fold(a_p[i],      &plo, &phi);
    if (vis[V_I])      fold(a_i[i],      &plo, &phi);
    if (vis[V_D])      fold(a_d[i],      &plo, &phi);
    if (plo <= phi) {
      pmin = p_have ? LV_MIN(pmin, plo) : plo;
      pmax = p_have ? LV_MAX(pmax, phi) : phi;
      p_have = true;
    }
  }

  // Pad the ranges: at least 0.1 degC beyond the temperature data, 5% + a
  // floor on the power pane, which always includes zero.
  int32_t tpad = LV_MAX(10, (tmax - tmin) / 20);
  tmin -= tpad; tmax += tpad;
  pmin = LV_MIN(pmin, 0); pmax = LV_MAX(pmax, 100);
  int32_t ppad = LV_MAX(20, (pmax - pmin) / 20);
  pmin -= (pmin < 0) ? ppad : 0; pmax += ppad;
  if (!t_have) { tmin = 0; tmax = 10000; }
  if (!p_have) { pmin = 0; pmax = 1000; }

  // Event markers: short dashes at the top edge on grain / mode transitions.
  for (int k = 1; k < n; k++) {
    int i = pts - 1 - (int)((now - snap[k].t_s) / 0.5);
    if (i < 1 || i >= pts) continue;
    if (snap[k].grain_in != snap[k - 1].grain_in)
      a_grain_ev[i - 1] = a_grain_ev[i] = tmax;
    if (snap[k].manual != snap[k - 1].manual)
      a_mode_ev[i - 1] = a_mode_ev[i] = tmax;
  }

  lv_chart_set_axis_range(S.chart_temp, LV_CHART_AXIS_PRIMARY_Y, tmin, tmax);
  lv_chart_set_axis_range(S.chart_pow, LV_CHART_AXIS_PRIMARY_Y, pmin, pmax);
  lv_chart_refresh(S.chart_temp);
  lv_chart_refresh(S.chart_pow);

  char b[24];
  snprintf(b, sizeof(b), "%.1f", tmax / 100.0f);
  lv_label_set_text(S.ylab_temp[0], b);
  snprintf(b, sizeof(b), "%.1f", (tmin + tmax) / 200.0f);
  lv_label_set_text(S.ylab_temp[1], b);
  snprintf(b, sizeof(b), "%.1f", tmin / 100.0f);
  lv_label_set_text(S.ylab_temp[2], b);
  snprintf(b, sizeof(b), "%d", (int)pmax);
  lv_label_set_text(S.ylab_pow[0], b);
  snprintf(b, sizeof(b), "%d", (int)((pmin + pmax) / 2));
  lv_label_set_text(S.ylab_pow[1], b);
  snprintf(b, sizeof(b), "%d", (int)pmin);
  lv_label_set_text(S.ylab_pow[2], b);
}

// ── Periodic refresh ──────────────────────────────────────────────────────────

static void refresh_cb(lv_timer_t *t) {
  (void)t;
  static int tick;
  tick++;

  float temp   = atomic_load(&ST->temp_filt_c);
  float sp     = atomic_load(&ST->setpoint_c);
  bool  manual = atomic_load(&ST->manual_mode);
  float demand = atomic_load(&ST->p_demand_w);
  bool  valid  = atomic_load(&ST->temp_valid);
  char  txt[64];

  // LVGL's own printf has no float support; format with libc instead.
  snprintf(txt, sizeof(txt), "Set %.1f°C", (double)sp);
  lv_label_set_text(S.lbl_setpoint, txt);
  if (valid) {
    snprintf(txt, sizeof(txt), "%.2f°C", (double)temp);
    lv_label_set_text(S.lbl_temp_c, txt);
    snprintf(txt, sizeof(txt), "%.1f°F", (double)(temp * 9.0f / 5.0f + 32.0f));
    lv_label_set_text(S.lbl_temp_f, txt);
  } else {
    lv_label_set_text(S.lbl_temp_c, "--.--°C");
    lv_label_set_text(S.lbl_temp_f, "--.-°F");
  }

  // Element glow tracks the commanded duty (the fired-state average), eased
  // with a one-pole IIR so power changes bloom rather than snap.
  float d1 = atomic_load(&ST->duty1), d2 = atomic_load(&ST->duty2);
  bool wdog_now = atomic_load(&ST->watchdog_alarm);
  float g1 = wdog_now ? 0.0f : d1, g2 = wdog_now ? 0.0f : d2;
  const float damp = 0.35f;
  S.bright1 += damp * (g1 - S.bright1);
  S.bright2 += damp * (g2 - S.bright2);
  bool grain = atomic_load(&ST->grain_in);
  ui_kettle_update(S.bright1, S.bright2, grain);

  snprintf(txt, sizeof(txt), "E1 %.0f%%   E2 %.0f%%",
           (double)(d1 * 100.0f), (double)(d2 * 100.0f));
  lv_label_set_text(S.lbl_elements, txt);

  // Slider: the user's command in manual; a live view of the loop in auto.
  if (manual != S.was_manual) {
    S.was_manual = manual;
    if (manual) {
      lv_obj_remove_state(S.slider, LV_STATE_DISABLED);
      lv_slider_set_value(S.slider, (int32_t)atomic_load(&ST->manual_power_w),
                          LV_ANIM_OFF);
    } else {
      lv_obj_add_state(S.slider, LV_STATE_DISABLED);
    }
  }
  if (!manual)
    lv_slider_set_value(S.slider, (int32_t)demand, LV_ANIM_OFF);
  snprintf(txt, sizeof(txt), "%d W", (int)demand);
  lv_label_set_text(S.slider_watts, txt);

  // Keep the toggle states honest (the sampler may force manual on a fault).
  if (manual) lv_obj_add_state(S.btn_manual, LV_STATE_CHECKED);
  else        lv_obj_remove_state(S.btn_manual, LV_STATE_CHECKED);
  if (grain)  lv_obj_add_state(S.btn_grain, LV_STATE_CHECKED);
  else        lv_obj_remove_state(S.btn_grain, LV_STATE_CHECKED);

  // Fault band.
  uint8_t fault_bits   = atomic_load(&ST->rtd_fault);
  bool    unresponsive = atomic_load(&ST->rtd_unresponsive);
  bool    watchdog     = atomic_load(&ST->watchdog_alarm);
  bool    forced       = atomic_load(&ST->fault_forced_manual);
  if (fault_bits || unresponsive || watchdog) {
    char msg[256];
    size_t used = 0;
    if (watchdog)
      used += (size_t)snprintf(msg + used, sizeof(msg) - used,
                               "WATCHDOG: no zero-cross. ");
    if (unresponsive)
      used += (size_t)snprintf(msg + used, sizeof(msg) - used,
                               "RTD not responding. ");
    else if (fault_bits) {
      char ft[128];
      max31865_fault_text(fault_bits, ft, sizeof(ft));
      used += (size_t)snprintf(msg + used, sizeof(msg) - used,
                               "RTD fault 0x%02X: %s. ", fault_bits, ft);
    }
    if (forced && used < sizeof(msg))
      snprintf(msg + used, sizeof(msg) - used, "Auto disabled -> manual.");
    lv_label_set_text(S.lbl_fault, msg);
    lv_obj_remove_flag(S.fault_band, LV_OBJ_FLAG_HIDDEN);
  } else {
    lv_obj_add_flag(S.fault_band, LV_OBJ_FLAG_HIDDEN);
  }

  // Status line: adaptive estimate and gain scale.
  float mc = atomic_load(&ST->mc_est_j_per_c);
  float scale = atomic_load(&ST->adaptive_scale);
  if (mc > 0.0f) {
    snprintf(txt, sizeof(txt), "mc %.0f kJ/°C  x%.2f",
             (double)(mc / 1000.0f), (double)scale);
    lv_label_set_text(S.lbl_status, txt);
  } else {
    lv_label_set_text(S.lbl_status, "");
  }

  if (tick % 2 == 0) update_charts();
}

// ── Screen construction ───────────────────────────────────────────────────────

void ui_screen_create(BibbyState *st, const BibbyConfig *cfg) {
  ST  = st;
  CFG = cfg;

  const PowerSplitConfig split_cfg = {
    cfg->element1_watts, cfg->element2_watts,
    cfg->element1_area_cm2, cfg->element2_area_cm2,
    cfg->max_flux_w_cm2,
  };
  S.max_watts = power_split_max_w(&split_cfg);
  float mins  = clampf(cfg->ui_chart_window_min, 1.0f, 30.0f);
  S.chart_pts = (int)(mins * 120.0f);
  S.was_manual = true;

  lv_obj_t *scr = lv_screen_active();
  lv_obj_set_style_bg_color(scr, lv_color_hex(UI_BG), 0);
  lv_obj_set_style_text_color(scr, lv_color_hex(UI_TEXT), 0);
  lv_obj_remove_flag(scr, LV_OBJ_FLAG_SCROLLABLE);

  // ── Toggles (top-left) ──
  bool zc_sim = cfg->ui_show_zc_sim;
  S.btn_zc_sim = zc_sim ? make_toggle(scr, "ZC Sim", 10, 10, 180, 75,
                                      UI_BLUE, zc_sim_toggle_cb)
                        : NULL;
  S.btn_grain  = make_toggle(scr, "Grain In", zc_sim ? 200 : 10, 10,
                             zc_sim ? 180 : 370, 75, UI_AMBER, grain_toggle_cb);
  S.btn_manual = make_toggle(scr, "Manual Control", 10, 95, 370, 75,
                             UI_GREEN, manual_toggle_cb);
  lv_obj_add_state(S.btn_manual, LV_STATE_CHECKED);  // manual is the default

  // ── Setpoint steppers ──
  static const float steps[6] = { 10.0f, 1.0f, 0.1f, -10.0f, -1.0f, -0.1f };
  static const char *step_lbl[6] = { "+10", "+1", "+0.1", "-10", "-1", "-0.1" };
  for (int i = 0; i < 6; i++) {
    lv_obj_t *b = lv_button_create(scr);
    lv_obj_set_pos(b, 400 + (i % 3) * 128, 10 + (i / 3) * 85);
    lv_obj_set_size(b, 120, 75);
    lv_obj_set_style_bg_color(b, lv_color_hex(UI_PANEL_HI), 0);
    lv_obj_set_style_radius(b, 10, 0);
    lv_obj_set_style_shadow_width(b, 0, 0);
    lv_obj_t *l = lv_label_create(b);
    lv_label_set_text(l, step_lbl[i]);
    lv_obj_set_style_text_font(l, &lv_font_montserrat_24, 0);
    lv_obj_center(l);
    lv_obj_add_event_cb(b, step_cb, LV_EVENT_CLICKED, (void *)&steps[i]);
  }

  // ── Charts ──
  S.chart_temp = make_chart(scr, 70, 205, 710, 245);
  S.chart_pow  = make_chart(scr, 70, 483, 710, 175);

  for (int i = 0; i < 3; i++) {
    S.ylab_temp[i] = make_label(scr, 4, 205 + i * 110, &lv_font_montserrat_14,
                                UI_TEXT_DIM, "");
    lv_obj_set_width(S.ylab_temp[i], 62);
    lv_obj_set_style_text_align(S.ylab_temp[i], LV_TEXT_ALIGN_RIGHT, 0);
    S.ylab_pow[i] = make_label(scr, 4, 483 + i * 76, &lv_font_montserrat_14,
                               UI_TEXT_DIM, "");
    lv_obj_set_width(S.ylab_pow[i], 62);
    lv_obj_set_style_text_align(S.ylab_pow[i], LV_TEXT_ALIGN_RIGHT, 0);
  }

  S.s_set      = add_series(S.chart_temp, UI_GREEN, a_set);
  S.s_raw      = add_series(S.chart_temp, 0x55606c, a_raw);
  S.s_filt     = add_series(S.chart_temp, UI_TRACE, a_filt);
  S.s_grain_ev = add_series(S.chart_temp, UI_AMBER, a_grain_ev);
  S.s_mode_ev  = add_series(S.chart_temp, UI_BLUE, a_mode_ev);

  S.s_deliv  = add_series(S.chart_pow, 0xa06428, a_deliv);
  S.s_ff     = add_series(S.chart_pow, UI_TEAL, a_ff);
  S.s_p      = add_series(S.chart_pow, UI_BLUE, a_p);
  S.s_i      = add_series(S.chart_pow, UI_VIOLET, a_i);
  S.s_d      = add_series(S.chart_pow, 0x9aa4b0, a_d);
  S.s_demand = add_series(S.chart_pow, UI_ORANGE, a_demand);

  // ── Legends: tap a chip to drop that trace off its chart ──
  lv_obj_t *leg1 = make_legend_row(scr, 70, 173, 710);
  make_chip(leg1, V_SET,   S.chart_temp, &S.s_set,      UI_GREEN, "setpoint");
  make_chip(leg1, V_FILT,  S.chart_temp, &S.s_filt,     UI_TRACE, "temp");
  make_chip(leg1, V_RAW,   S.chart_temp, &S.s_raw,      0x9aa4b0, "raw");
  // The two event traces are dashes along the top edge, not curves: they mark
  // the samples where Grain In / Manual flipped.
  make_chip(leg1, V_GRAIN, S.chart_temp, &S.s_grain_ev, UI_AMBER, "grain event");
  make_chip(leg1, V_MODE,  S.chart_temp, &S.s_mode_ev,  UI_BLUE,  "mode event");

  lv_obj_t *leg2 = make_legend_row(scr, 70, 451, 710);
  make_chip(leg2, V_DEMAND, S.chart_pow, &S.s_demand, UI_ORANGE, "demand W");
  make_chip(leg2, V_DELIV,  S.chart_pow, &S.s_deliv,  0xc4823f,  "delivered");
  make_chip(leg2, V_FF,     S.chart_pow, &S.s_ff,     UI_TEAL,   "ff");
  make_chip(leg2, V_P,      S.chart_pow, &S.s_p,      UI_BLUE,   "P");
  make_chip(leg2, V_I,      S.chart_pow, &S.s_i,      UI_VIOLET, "I");
  make_chip(leg2, V_D,      S.chart_pow, &S.s_d,      0x9aa4b0,  "D");

  // ── Fault band + status (bottom-left) ──
  S.fault_band = make_panel(scr, 10, 668, 590, 46);
  lv_obj_set_style_bg_color(S.fault_band, lv_color_hex(0x3a1512), 0);
  lv_obj_set_style_border_color(S.fault_band, lv_color_hex(UI_RED), 0);
  S.lbl_fault = make_label(S.fault_band, 10, 4, &lv_font_montserrat_14, UI_RED, "");
  lv_label_set_long_mode(S.lbl_fault, LV_LABEL_LONG_WRAP);
  lv_obj_set_width(S.lbl_fault, 570);
  lv_obj_add_flag(S.fault_band, LV_OBJ_FLAG_HIDDEN);
  S.lbl_status = make_label(scr, 610, 684, &lv_font_montserrat_14, UI_TEXT_DIM, "");

  // ── Power slider ──
  make_label(scr, 806, 14, &lv_font_montserrat_16, UI_TEXT_DIM, "POWER");
  S.slider = lv_slider_create(scr);
  lv_obj_set_pos(S.slider, 812, 75);   // knob overhang must clear "POWER"
  lv_obj_set_size(S.slider, 56, 570);
  lv_slider_set_range(S.slider, 0, (int32_t)S.max_watts);
  lv_slider_set_value(S.slider, (int32_t)atomic_load(&st->manual_power_w),
                      LV_ANIM_OFF);
  lv_obj_set_style_bg_color(S.slider, lv_color_hex(UI_PANEL_HI), LV_PART_MAIN);
  lv_obj_set_style_bg_color(S.slider, lv_color_hex(UI_ORANGE), LV_PART_INDICATOR);
  lv_obj_set_style_bg_color(S.slider, lv_color_hex(UI_TEXT), LV_PART_KNOB);
  lv_obj_add_event_cb(S.slider, slider_cb, LV_EVENT_VALUE_CHANGED, NULL);
  S.slider_watts = make_label(scr, 790, 692, &lv_font_montserrat_20, UI_TEXT, "0 W");
  lv_obj_set_width(S.slider_watts, 100);
  lv_obj_set_style_text_align(S.slider_watts, LV_TEXT_ALIGN_CENTER, 0);

  // ── Kettle column ──
  S.lbl_setpoint = make_label(scr, 900, 10, &lv_font_montserrat_32, UI_GREEN, "");
  lv_obj_set_width(S.lbl_setpoint, 370);
  lv_obj_set_style_text_align(S.lbl_setpoint, LV_TEXT_ALIGN_CENTER, 0);

  lv_obj_t *kettle = ui_kettle_create(scr, 370, 560);
  lv_obj_set_pos(kettle, 900, 55);

  S.lbl_temp_c = make_label(scr, 900, 130, &lv_font_montserrat_48, UI_TEXT, "");
  lv_obj_set_width(S.lbl_temp_c, 370);
  lv_obj_set_style_text_align(S.lbl_temp_c, LV_TEXT_ALIGN_CENTER, 0);
  S.lbl_temp_f = make_label(scr, 900, 185, &lv_font_montserrat_32, UI_TEXT_DIM, "");
  lv_obj_set_width(S.lbl_temp_f, 370);
  lv_obj_set_style_text_align(S.lbl_temp_f, LV_TEXT_ALIGN_CENTER, 0);

  S.lbl_elements = make_label(scr, 900, 625, &lv_font_montserrat_20, UI_TEXT_DIM, "");
  lv_obj_set_width(S.lbl_elements, 370);
  lv_obj_set_style_text_align(S.lbl_elements, LV_TEXT_ALIGN_CENTER, 0);

  lv_timer_create(refresh_cb, 250, NULL);
  refresh_cb(NULL);
}

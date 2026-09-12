// Plain-assert unit tests for the pure (hardware-free) bibby modules.
// Build target: `tests` — runs on any Linux host, no GPIO/SPI needed.
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "config.h"
#include "control.h"
#include "mass_estimator.h"
#include "power_split.h"
#include "sigma_delta.h"
#include "temp_filter.h"

static int failures = 0;

#define CHECK(cond) do { \
  if (!(cond)) { \
    fprintf(stderr, "FAIL %s:%d  %s\n", __FILE__, __LINE__, #cond); \
    failures++; \
  } \
} while (0)

#define CHECK_NEAR(a, b, tol) do { \
  double _a = (a), _b = (b); \
  if (fabs(_a - _b) > (tol)) { \
    fprintf(stderr, "FAIL %s:%d  %s=%.6g vs %s=%.6g (tol %.3g)\n", \
            __FILE__, __LINE__, #a, _a, #b, _b, (double)(tol)); \
    failures++; \
  } \
} while (0)

// ── power_split ──────────────────────────────────────────────────────────────

static void test_power_split(void) {
  // Identical elements: both at the same duty, p/(P1+P2).
  PowerSplitConfig eq = { 2500, 2500, 150, 150, 0 };
  PowerSplitResult r = power_split(&eq, 2500);
  CHECK_NEAR(r.duty1, 0.5, 1e-6);
  CHECK_NEAR(r.duty2, 0.5, 1e-6);
  CHECK_NEAR(r.x1_w + r.x2_w, 2500, 1e-3);

  // Unequal elements, no saturation: equal flux on both.
  PowerSplitConfig uneq = { 5000, 5500, 820, 473, 0 };
  r = power_split(&uneq, 4000);
  CHECK_NEAR(r.flux1_w_cm2, r.flux2_w_cm2, 1e-5);
  CHECK_NEAR(r.x1_w + r.x2_w, 4000, 1e-2);

  // Saturation: element 2 (small area, would exceed rating at equal flux for
  // a config where its share tops out) — force with a skewed config.
  PowerSplitConfig skew = { 5000, 1000, 500, 500, 0 };
  r = power_split(&skew, 5000);           // equal flux wants 2500 each; P2 caps at 1000
  CHECK_NEAR(r.x2_w, 1000, 1e-3);
  CHECK_NEAR(r.x1_w, 4000, 1e-3);
  CHECK_NEAR(r.duty2, 1.0, 1e-6);

  // Demand above total: clamp to P1+P2, both at full duty.
  r = power_split(&uneq, 99999);
  CHECK_NEAR(r.duty1, 1.0, 1e-6);
  CHECK_NEAR(r.duty2, 1.0, 1e-6);

  // Negative demand: zero.
  r = power_split(&uneq, -100);
  CHECK_NEAR(r.x1_w + r.x2_w, 0.0, 1e-9);

  // Flux cap: max deliverable power limited to max_flux * total area.
  PowerSplitConfig cap = { 5000, 5500, 820, 473, 5.0f };
  CHECK_NEAR(power_split_max_w(&cap), 5.0 * (820 + 473), 1e-3);
  r = power_split(&cap, 99999);
  CHECK_NEAR(r.flux1_w_cm2, 5.0, 1e-3);
  CHECK_NEAR(r.flux2_w_cm2, 5.0, 1e-3);

  // Degenerate config must not divide by zero.
  PowerSplitConfig zero = { 0, 0, 0, 0, 0 };
  r = power_split(&zero, 1000);
  CHECK(r.duty1 == 0.0f && r.duty2 == 0.0f);
}

// ── sigma_delta ──────────────────────────────────────────────────────────────

static void test_sigma_delta(void) {
  const float duties[] = { 0.0f, 0.1f, 1.0f / 3.0f, 0.5f, 0.9f, 1.0f };
  for (unsigned k = 0; k < sizeof(duties) / sizeof(duties[0]); k++) {
    SigmaDelta sd = {0};
    int fired = 0, n = 12000;
    for (int i = 0; i < n; i++)
      if (sigma_delta_step(&sd, duties[k])) fired++;
    CHECK_NEAR((double)fired / n, duties[k], 1.0 / n + 1e-9);
  }

  // duty 0 never fires; duty 1 always fires.
  SigmaDelta sd = {0};
  for (int i = 0; i < 100; i++) CHECK(!sigma_delta_step(&sd, 0.0f));
  sd.acc = 0;
  for (int i = 0; i < 100; i++) CHECK(sigma_delta_step(&sd, 1.0f));

  // Accumulator cap bounds catch-up after a forced-off stretch.
  sd.acc = 5.0f;  // impossible normally; the cap must pull it back
  sigma_delta_step(&sd, 1.0f);
  CHECK(sd.acc <= 2.0f);
}

// ── control ──────────────────────────────────────────────────────────────────

static void test_control(void) {
  Pid pid;

  // Zero gains: output equals the (clamped) feedforward.
  pid_init(&pid, 0, 0, 0);
  CHECK_NEAR(pid_update(&pid, 60, 65, 1200, 10000, 1.0f), 1200, 1e-3);
  CHECK_NEAR(pid_update(&pid, 60, 65, 20000, 10000, 1.0f), 10000, 1e-3);

  // P-only: kp * error, clamped at zero from below.
  pid_init(&pid, 100, 0, 0);
  CHECK_NEAR(pid_update(&pid, 60, 65, 0, 10000, 1.0f), 500, 1e-3);
  CHECK_NEAR(pid_update(&pid, 70, 65, 0, 10000, 1.0f), 0, 1e-9);

  // gain_scale multiplies kp.
  pid_init(&pid, 100, 0, 0);
  CHECK_NEAR(pid_update(&pid, 60, 65, 0, 10000, 2.0f), 1000, 1e-3);

  // Anti-windup: with the output railed high and error still positive, the
  // integrator must hold.
  pid_init(&pid, 1000, 1.0f, 0);
  pid_update(&pid, 20, 65, 0, 5000, 1.0f);   // p = 45000 >> rail
  float wound = pid.integral;
  pid_update(&pid, 20, 65, 0, 5000, 1.0f);
  CHECK(pid.integral == wound);              // held, not accumulating

  // Integrator accumulates when not railed, and i-term tracks ki*integral.
  pid_init(&pid, 0, 10.0f, 0);
  pid_update(&pid, 64, 65, 0, 10000, 1.0f);  // error 1
  pid_update(&pid, 64, 65, 0, 10000, 1.0f);
  CHECK_NEAR(pid.terms.i_w, 10.0 * 2.0, 1e-3);

  // Integral backstop: bounded to full authority out_max/ki.
  pid_init(&pid, 0, 2.0f, 0);
  for (int i = 0; i < 100000; i++) pid_update(&pid, 64.9f, 65, 0, 1000, 1.0f);
  CHECK(pid.terms.i_w <= 1000.0f + 1e-3);

  // Integral separation: outside the band the integrator holds; inside it
  // accumulates; a gain-set swap keeps the band.
  pid_init(&pid, 0, 10.0f, 0);
  pid_set_i_band(&pid, 0.5f);
  pid_update(&pid, 60, 65, 0, 10000, 1.0f);   // error 5 > band: hold
  CHECK(pid.integral == 0.0f);
  pid_update(&pid, 64.7f, 65, 0, 10000, 1.0f); // error 0.3 <= band: accumulate
  CHECK_NEAR(pid.integral, 0.3, 1e-4);
  pid_update(&pid, 66, 65, 0, 10000, 1.0f);   // error -1 outside: hold, no reset
  CHECK_NEAR(pid.integral, 0.3, 1e-4);
  pid_set_gains(&pid, 0, 5.0f, 0);
  CHECK_NEAR(pid.i_band_c, 0.5, 1e-6);
  pid_set_i_band(&pid, -1.0f);                 // negative clamps to off
  CHECK(pid.i_band_c == 0.0f);
  pid_update(&pid, 60, 65, 0, 10000, 1.0f);   // band off: integrates again
  CHECK_NEAR(pid.integral, 5.0, 1e-4);

  // Derivative: kd * per-sample error delta; no kick on the first sample.
  pid_init(&pid, 0, 0, 50);
  pid_update(&pid, 60, 65, 0, 10000, 1.0f);
  CHECK_NEAR(pid.terms.d_w, 0, 1e-9);
  pid_update(&pid, 61, 65, 0, 10000, 1.0f);  // error 5 -> 4
  CHECK_NEAR(pid.terms.d_w, 50.0 * -1.0, 1e-3);

  // Feedforward helper: disabled at gain 0, clamped, correct slope.
  CHECK_NEAR(control_feedforward_w(65, 20, 0.0f, 10000), 0, 1e-9);
  CHECK_NEAR(control_feedforward_w(65, 20, 0.009f, 10000), 45.0 / 0.009, 1e-1);
  CHECK_NEAR(control_feedforward_w(200, 20, 0.001f, 10000), 10000, 1e-3);
  CHECK_NEAR(control_feedforward_w(10, 20, 0.009f, 10000), 0, 1e-9);
}

// ── temp_filter ──────────────────────────────────────────────────────────────

static void test_temp_filter(void) {
  TempFilter f;

  // Priming: first sample comes straight through; constant input stays put.
  CHECK(temp_filter_init(&f, 2, 40) == 0);
  CHECK_NEAR(temp_filter_push(&f, 25.0f), 25.0, 1e-6);
  for (int i = 0; i < 100; i++) CHECK_NEAR(temp_filter_push(&f, 25.0f), 25.0, 1e-4);
  temp_filter_free(&f);

  // Step settles to the final value within order*window samples.
  CHECK(temp_filter_init(&f, 2, 40) == 0);
  temp_filter_push(&f, 0.0f);
  float v = 0;
  for (int i = 0; i < 80; i++) v = temp_filter_push(&f, 10.0f);
  CHECK_NEAR(v, 10.0, 1e-3);
  temp_filter_free(&f);

  // Order 1, window 1: passthrough.
  CHECK(temp_filter_init(&f, 1, 1) == 0);
  temp_filter_push(&f, 1.0f);
  CHECK_NEAR(temp_filter_push(&f, 7.5f), 7.5, 1e-6);
  temp_filter_free(&f);

  // Group delay bookkeeping.
  CHECK(temp_filter_init(&f, 2, 40) == 0);
  CHECK_NEAR(temp_filter_group_delay(&f), 39.0, 1e-6);
  temp_filter_free(&f);
}

// ── mass_estimator ───────────────────────────────────────────────────────────

static void test_mass_estimator(void) {
  // Constant 3000 W into 20 L of water (m*c = 83.72 kJ/degC): slope
  // 0.03583 degC/s. Feed 5 minutes at 10 Hz; the estimate should converge
  // near 20 L.
  MassEstimator e;
  mass_estimator_init(&e, 200.0f);
  double litres_true = 20.0;
  double mc_true = litres_true * WATER_C_J_PER_KG_C;
  for (int i = 0; i < 3000; i++) {
    double t = i * 0.1;
    mass_estimator_push(&e, t, (float)(20.0 + 3000.0 / mc_true * t), 3000.0f);
  }
  CHECK(mass_estimator_value(&e) > 0);
  CHECK_NEAR(mass_estimator_value(&e), litres_true, litres_true * 0.05);

  // Wildly varying power: no estimate accepted.
  mass_estimator_init(&e, 200.0f);
  for (int i = 0; i < 3000; i++) {
    double t = i * 0.1;
    float p = (i / 100) % 2 ? 4000.0f : 500.0f;
    mass_estimator_push(&e, t, (float)(20.0 + 0.03 * t), p);
  }
  CHECK(mass_estimator_value(&e) == 0.0f);

  // Power below the floor: no estimate.
  mass_estimator_init(&e, 200.0f);
  for (int i = 0; i < 3000; i++)
    mass_estimator_push(&e, i * 0.1, (float)(20.0 + 0.05 * i * 0.1), 100.0f);
  CHECK(mass_estimator_value(&e) == 0.0f);
}

// ── config ───────────────────────────────────────────────────────────────────

static void test_config(void) {
  BibbyConfig cfg;

  // Missing file: defaults stand, load reports failure.
  CHECK(!config_load(&cfg, "/nonexistent/bibby.ini"));
  CHECK(cfg.mains_hz == 60);
  CHECK(cfg.pid_kp == 0.0f);
  CHECK(cfg.pid_i_band_c == 0.0f);
  CHECK(cfg.log_high_rate);
  CHECK(cfg.ui_show_zc_sim);
  CHECK(cfg.ui_rotation == 90);
  CHECK(!strcmp(cfg.ui_fb_device, "auto"));

  // Derived values.
  CHECK(config_zc_timeout_ns(&cfg) == 1000000000ULL / 120 + 700000ULL);
  CHECK(config_staleness_zc(&cfg) == 120);
  cfg.mains_hz = 50;
  CHECK(config_zc_timeout_ns(&cfg) == 1000000000ULL / 100 + 700000ULL);
  CHECK(config_staleness_zc(&cfg) == 100);

  // Round-trip a full file, including comments, booleans, and strings.
  const char *path = "/tmp/bibby_test.ini";
  FILE *f = fopen(path, "w");
  fprintf(f,
    "[mains]\nfrequency_hz = 50\n"
    "[element1]\nwatts = 5000 ; comment\narea_cm2 = 820\n"
    "[element2]\nwatts = 5500\narea_cm2 = 473\n"
    "[power]\nmax_flux_w_cm2 = 4.5\n"
    "[pid]\nkp = 350\nki = 0.02\nkd = 10\ni_band_c = 0.75\n"
    "[grain]\nkp = 200\nki = 0.01\nkd = 5\nmax_power_w = 6000\n"
    "[feedforward]\nprocess_gain_c = 0.011\nambient_c = 18\n"
    "[sensor]\nref_resistor_ohms = 397.82\ntemp_cal_gain = 1.0528\ntemp_cal_offset = -0.032\n"
    "[filter]\norder = 3\nwindow = 20\n"
    "[logging]\nrate = low\nlow_period_s = 1.5\n"
    "[ui]\nshow_zc_sim = false\nchart_window_min = 10\nrotation = 270\nfb_device = /dev/fb1\ntouch_device = /dev/input/event1\n"
    "[adaptive]\nenable = true\nm_ref_l = 45\nscale_min = 0.6\nscale_max = 3\n");
  fclose(f);

  CHECK(config_load(&cfg, path));
  CHECK(cfg.mains_hz == 50);
  CHECK_NEAR(cfg.element1_watts, 5000, 1e-6);
  CHECK_NEAR(cfg.element2_area_cm2, 473, 1e-6);
  CHECK_NEAR(cfg.max_flux_w_cm2, 4.5, 1e-6);
  CHECK_NEAR(cfg.pid_kp, 350, 1e-6);
  CHECK_NEAR(cfg.pid_i_band_c, 0.75, 1e-6);
  CHECK_NEAR(cfg.grain_max_power_w, 6000, 1e-6);
  CHECK_NEAR(cfg.ff_process_gain_c, 0.011, 1e-9);
  CHECK_NEAR(cfg.sensor_ref_resistor_ohms, 397.82, 1e-4);
  CHECK(cfg.filter_order == 3 && cfg.filter_window == 20);
  CHECK(!cfg.log_high_rate);
  CHECK_NEAR(cfg.log_low_period_s, 1.5, 1e-6);
  CHECK(!cfg.ui_show_zc_sim);
  CHECK(cfg.ui_rotation == 270);
  CHECK(!strcmp(cfg.ui_fb_device, "/dev/fb1"));
  CHECK(!strcmp(cfg.ui_touch_device, "/dev/input/event1"));
  CHECK(cfg.adaptive_enable);
  CHECK_NEAR(cfg.adaptive_m_ref_l, 45, 1e-3);
  CHECK_NEAR(config_total_watts(&cfg), 10500, 1e-3);
  remove(path);
}

int main(void) {
  test_power_split();
  test_sigma_delta();
  test_control();
  test_temp_filter();
  test_mass_estimator();
  test_config();

  if (failures) {
    fprintf(stderr, "%d test(s) FAILED\n", failures);
    return 1;
  }
  printf("all tests passed\n");
  return 0;
}

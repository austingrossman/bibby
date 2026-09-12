#include "config.h"

#include <ctype.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>

static char resolved_path[512] = "bibby.ini";

const char *config_resolved_path(void) { return resolved_path; }

static void set_defaults(BibbyConfig *cfg) {
  cfg->mains_hz          = 60;
  cfg->element1_watts    = 2500.0f;
  cfg->element2_watts    = 2500.0f;
  cfg->element1_area_cm2 = 150.0f;
  cfg->element2_area_cm2 = 150.0f;
  cfg->max_flux_w_cm2    = 0.0f;   // anti-scorch cap disabled
  cfg->pid_kp            = 0.0f;   // zero gains: auto commands no power until tuned
  cfg->pid_ki            = 0.0f;
  cfg->pid_kd            = 0.0f;
  cfg->grain_kp          = 0.0f;
  cfg->grain_ki          = 0.0f;
  cfg->grain_kd          = 0.0f;
  cfg->grain_max_power_w = 0.0f;   // no extra grain-in power cap
  cfg->ff_process_gain_c = 0.0f;   // feedforward disabled until K is identified
  cfg->ff_ambient_c      = 20.0f;
  cfg->sensor_ref_resistor_ohms = 400.0f; // nominal Rref, no ice-point trim
  cfg->sensor_temp_cal_gain     = 1.0f;   // identity span (uncalibrated)
  cfg->sensor_temp_cal_offset   = 0.0f;
  cfg->filter_order      = 2;
  cfg->filter_window     = 40;
  cfg->log_high_rate     = true;
  cfg->log_low_period_s  = 2.0f;
  cfg->ui_show_zc_sim    = true;
  cfg->ui_chart_window_min = 5.0f;
  cfg->ui_rotation       = 90;
  snprintf(cfg->ui_fb_device,    sizeof(cfg->ui_fb_device),    "auto");
  snprintf(cfg->ui_touch_device, sizeof(cfg->ui_touch_device), "auto");
  cfg->web_enable        = false;  // off until a password is set in bibby.ini
  cfg->web_port          = 8080;
  cfg->web_allow_control = true;
  cfg->web_screen_fps    = 5;
  snprintf(cfg->web_bind,     sizeof(cfg->web_bind),     "0.0.0.0");
  snprintf(cfg->web_user,     sizeof(cfg->web_user),     "brewer");
  cfg->web_password[0]   = '\0';   // empty = server refuses to start
  cfg->pid_i_band_c           = 0.0f;   // off: classic PI until the ini sets it
  cfg->adaptive_enable        = false;
  cfg->adaptive_m_ref_l       = 0.0f;
  cfg->adaptive_scale_min     = 0.5f;
  cfg->adaptive_scale_max     = 4.0f;
}

// Trim leading/trailing ASCII whitespace in place; returns the new start.
static char *trim(char *s) {
  while (*s && isspace((unsigned char)*s)) s++;
  if (!*s) return s;
  char *end = s + strlen(s) - 1;
  while (end > s && isspace((unsigned char)*end)) *end-- = '\0';
  return s;
}

static bool parse_bool(const char *val) {
  return !strcmp(val, "1") || !strcmp(val, "true") || !strcmp(val, "yes") ||
         !strcmp(val, "on");
}

// Apply one section-qualified "key" = "val" pair (e.g. "pid.kp") to cfg.
// Unrecognized keys are silently ignored so the file can carry comments/extras.
static void apply(BibbyConfig *cfg, const char *key, const char *val) {
  if      (!strcmp(key, "mains.frequency_hz")) cfg->mains_hz          = atoi(val);
  else if (!strcmp(key, "element1.watts"))     cfg->element1_watts    = (float)atof(val);
  else if (!strcmp(key, "element2.watts"))     cfg->element2_watts    = (float)atof(val);
  else if (!strcmp(key, "element1.area_cm2"))  cfg->element1_area_cm2 = (float)atof(val);
  else if (!strcmp(key, "element2.area_cm2"))  cfg->element2_area_cm2 = (float)atof(val);
  else if (!strcmp(key, "power.max_flux_w_cm2")) cfg->max_flux_w_cm2  = (float)atof(val);
  else if (!strcmp(key, "pid.kp"))             cfg->pid_kp            = (float)atof(val);
  else if (!strcmp(key, "pid.ki"))             cfg->pid_ki            = (float)atof(val);
  else if (!strcmp(key, "pid.kd"))             cfg->pid_kd            = (float)atof(val);
  else if (!strcmp(key, "pid.i_band_c"))       cfg->pid_i_band_c      = (float)atof(val);
  else if (!strcmp(key, "grain.kp"))           cfg->grain_kp          = (float)atof(val);
  else if (!strcmp(key, "grain.ki"))           cfg->grain_ki          = (float)atof(val);
  else if (!strcmp(key, "grain.kd"))           cfg->grain_kd          = (float)atof(val);
  else if (!strcmp(key, "grain.max_power_w"))  cfg->grain_max_power_w = (float)atof(val);
  else if (!strcmp(key, "feedforward.process_gain_c")) cfg->ff_process_gain_c = (float)atof(val);
  else if (!strcmp(key, "feedforward.ambient_c"))      cfg->ff_ambient_c      = (float)atof(val);
  else if (!strcmp(key, "sensor.ref_resistor_ohms")) cfg->sensor_ref_resistor_ohms = (float)atof(val);
  else if (!strcmp(key, "sensor.temp_cal_gain"))     cfg->sensor_temp_cal_gain     = (float)atof(val);
  else if (!strcmp(key, "sensor.temp_cal_offset"))   cfg->sensor_temp_cal_offset   = (float)atof(val);
  else if (!strcmp(key, "filter.order"))       cfg->filter_order      = atoi(val);
  else if (!strcmp(key, "filter.window"))      cfg->filter_window     = atoi(val);
  else if (!strcmp(key, "logging.rate"))       cfg->log_high_rate     = !strcmp(val, "high");
  else if (!strcmp(key, "logging.low_period_s")) cfg->log_low_period_s = (float)atof(val);
  else if (!strcmp(key, "ui.show_zc_sim"))     cfg->ui_show_zc_sim    = parse_bool(val);
  else if (!strcmp(key, "ui.chart_window_min")) cfg->ui_chart_window_min = (float)atof(val);
  else if (!strcmp(key, "ui.rotation"))        cfg->ui_rotation       = atoi(val);
  else if (!strcmp(key, "ui.fb_device"))       snprintf(cfg->ui_fb_device, sizeof(cfg->ui_fb_device), "%s", val);
  else if (!strcmp(key, "ui.touch_device"))    snprintf(cfg->ui_touch_device, sizeof(cfg->ui_touch_device), "%s", val);
  else if (!strcmp(key, "web.enable"))          cfg->web_enable        = parse_bool(val);
  else if (!strcmp(key, "web.port"))            cfg->web_port          = atoi(val);
  else if (!strcmp(key, "web.bind"))            snprintf(cfg->web_bind,     sizeof(cfg->web_bind),     "%s", val);
  else if (!strcmp(key, "web.user"))            snprintf(cfg->web_user,     sizeof(cfg->web_user),     "%s", val);
  else if (!strcmp(key, "web.password"))        snprintf(cfg->web_password, sizeof(cfg->web_password), "%s", val);
  else if (!strcmp(key, "web.allow_control"))   cfg->web_allow_control = parse_bool(val);
  else if (!strcmp(key, "web.screen_fps"))      cfg->web_screen_fps    = atoi(val);
  else if (!strcmp(key, "adaptive.enable"))    cfg->adaptive_enable   = parse_bool(val);
  else if (!strcmp(key, "adaptive.m_ref_l"))   cfg->adaptive_m_ref_l   = (float)atof(val);
  else if (!strcmp(key, "adaptive.scale_min")) cfg->adaptive_scale_min = (float)atof(val);
  else if (!strcmp(key, "adaptive.scale_max")) cfg->adaptive_scale_max = (float)atof(val);
}

// bibby.ini in the directory holding the running executable, so an installed
// binary finds its config without a baked build-tree path. Returns false if
// /proc/self/exe cannot be resolved.
static bool exe_sibling_ini(char *out, size_t out_size) {
  char exe[480];  // leaves room for "/bibby.ini" in the caller's 512-byte path
  ssize_t n = readlink("/proc/self/exe", exe, sizeof(exe) - 1);
  if (n <= 0) return false;
  exe[n] = '\0';
  char *slash = strrchr(exe, '/');
  if (!slash) return false;
  *slash = '\0';
  snprintf(out, out_size, "%s/bibby.ini", exe);
  return true;
}

bool config_load(BibbyConfig *cfg, const char *path) {
  set_defaults(cfg);

  FILE *f = NULL;
  if (path && *path) {
    snprintf(resolved_path, sizeof(resolved_path), "%s", path);
    f = fopen(path, "r");
  } else {
    const char *env = getenv("BIBBY_CONFIG");
    if (env && *env) {
      snprintf(resolved_path, sizeof(resolved_path), "%s", env);
      f = fopen(env, "r");
    }
    if (!f && exe_sibling_ini(resolved_path, sizeof(resolved_path)))
      f = fopen(resolved_path, "r");
    if (!f) {
      snprintf(resolved_path, sizeof(resolved_path), "bibby.ini");
      f = fopen(resolved_path, "r");
    }
  }
  if (!f) return false;

  char line[256];
  char section[64] = "";
  while (fgets(line, sizeof(line), f)) {
    // Drop comments (# or ;) and surrounding whitespace.
    char *cmt = strpbrk(line, "#;");
    if (cmt) *cmt = '\0';
    char *s = trim(line);
    if (!*s) continue;

    if (*s == '[') {
      char *close = strchr(s, ']');
      if (close) {
        *close = '\0';
        snprintf(section, sizeof(section), "%s", trim(s + 1));
      }
      continue;
    }

    char *eq = strchr(s, '=');
    if (!eq) continue;
    *eq = '\0';
    char *key = trim(s);
    char *val = trim(eq + 1);

    char qualified[128];
    snprintf(qualified, sizeof(qualified), "%s.%s", section, key);
    apply(cfg, qualified, val);
  }
  fclose(f);
  return true;
}

uint64_t config_zc_timeout_ns(const BibbyConfig *cfg) {
  int hz = (cfg->mains_hz == 50) ? 50 : 60;
  // Zero crossings arrive every mains half-cycle; wait one half-period plus a
  // 0.7 ms guard so normal jitter does not trip the watchdog.
  uint64_t half_ns = 1000000000ULL / (2ULL * (uint64_t)hz);
  return half_ns + 700000ULL;
}

uint32_t config_staleness_zc(const BibbyConfig *cfg) {
  int hz = (cfg->mains_hz == 50) ? 50 : 60;
  return (uint32_t)(2 * hz);  // ~2 s worth of zero crossings
}

float config_total_watts(const BibbyConfig *cfg) {
  return cfg->element1_watts + cfg->element2_watts;
}

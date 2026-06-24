#include "config.h"

#include <ctype.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

// Compile-time default ini path. Shares its name with the $BIBBY_CONFIG runtime
// override below by design; the getenv() string literal is not macro-expanded.
#ifndef BIBBY_CONFIG
#define BIBBY_CONFIG "bibby.ini"
#endif

static void set_defaults(BibbyConfig *cfg) {
  cfg->mains_hz          = 60;
  cfg->element1_watts    = 2500.0f;
  cfg->element2_watts    = 2500.0f;
  cfg->element1_area_cm2 = 150.0f;
  cfg->element2_area_cm2 = 150.0f;
  cfg->pid_kp            = 0.0f;
  cfg->pid_ki            = 0.0f;
  cfg->pid_kd            = 0.0f;
  cfg->ff_process_gain_c = 0.0f;  // feedforward disabled until K is set
  cfg->ff_ambient_c      = 20.0f; // nominal room temperature
  cfg->sensor_ref_resistor_ohms = 400.0f; // nominal Rref, no ice-point trim
  cfg->sensor_temp_cal_gain     = 1.0f;   // identity span (uncalibrated)
  cfg->sensor_temp_cal_offset   = 0.0f;
}

// Trim leading/trailing ASCII whitespace in place; returns the new start.
static char *trim(char *s) {
  while (*s && isspace((unsigned char)*s)) s++;
  if (!*s) return s;
  char *end = s + strlen(s) - 1;
  while (end > s && isspace((unsigned char)*end)) *end-- = '\0';
  return s;
}

// Apply one section-qualified "key" = "val" pair (e.g. "pid.kp") to cfg.
// Unrecognized keys are silently ignored so the file can carry comments/extras.
static void apply(BibbyConfig *cfg, const char *key, const char *val) {
  if      (!strcmp(key, "mains.frequency_hz")) cfg->mains_hz          = atoi(val);
  else if (!strcmp(key, "element1.watts"))     cfg->element1_watts    = (float)atof(val);
  else if (!strcmp(key, "element2.watts"))     cfg->element2_watts    = (float)atof(val);
  else if (!strcmp(key, "element1.area_cm2"))  cfg->element1_area_cm2 = (float)atof(val);
  else if (!strcmp(key, "element2.area_cm2"))  cfg->element2_area_cm2 = (float)atof(val);
  else if (!strcmp(key, "pid.kp"))             cfg->pid_kp            = (float)atof(val);
  else if (!strcmp(key, "pid.ki"))             cfg->pid_ki            = (float)atof(val);
  else if (!strcmp(key, "pid.kd"))             cfg->pid_kd            = (float)atof(val);
  else if (!strcmp(key, "feedforward.process_gain_c")) cfg->ff_process_gain_c = (float)atof(val);
  else if (!strcmp(key, "feedforward.ambient_c"))      cfg->ff_ambient_c      = (float)atof(val);
  else if (!strcmp(key, "sensor.ref_resistor_ohms")) cfg->sensor_ref_resistor_ohms = (float)atof(val);
  else if (!strcmp(key, "sensor.temp_cal_gain"))     cfg->sensor_temp_cal_gain     = (float)atof(val);
  else if (!strcmp(key, "sensor.temp_cal_offset"))   cfg->sensor_temp_cal_offset   = (float)atof(val);
}

bool config_load(BibbyConfig *cfg, const char *path) {
  set_defaults(cfg);

  if (!path || !*path) path = getenv("BIBBY_CONFIG");
  if (!path || !*path) path = BIBBY_CONFIG;

  FILE *f = fopen(path, "r");
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
  // 0.7 ms guard so normal jitter does not trip the watchdog. 60 Hz -> ~9.0 ms,
  // 50 Hz -> ~10.7 ms.
  uint64_t half_ns = 1000000000ULL / (2ULL * (uint64_t)hz);
  return half_ns + 700000ULL;
}

uint32_t config_frontend_watchdog_zc(const BibbyConfig *cfg) {
  int hz = (cfg->mains_hz == 50) ? 50 : 60;
  return (uint32_t)(2 * hz);  // ~2 s worth of zero crossings
}

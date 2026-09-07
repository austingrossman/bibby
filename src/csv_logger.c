#include "csv_logger.h"

#include <errno.h>
#include <stdlib.h>
#include <string.h>
#include <sys/stat.h>
#include <time.h>

// Create every component of `dir` (like `mkdir -p`). Returns 0 on success.
static int mkdir_p(const char *dir) {
  char tmp[512];
  snprintf(tmp, sizeof(tmp), "%s", dir);
  size_t n = strlen(tmp);
  if (n == 0) return -1;
  if (tmp[n - 1] == '/') tmp[n - 1] = '\0';

  for (char *p = tmp + 1; *p; p++) {
    if (*p != '/') continue;
    *p = '\0';
    if (mkdir(tmp, 0755) != 0 && errno != EEXIST) return -1;
    *p = '/';
  }
  if (mkdir(tmp, 0755) != 0 && errno != EEXIST) return -1;
  return 0;
}

int csv_logger_open(CsvLogger *lg, bool high_rate, float low_period_s) {
  memset(lg, 0, sizeof(*lg));
  lg->high_rate    = high_rate;
  lg->low_period_s = low_period_s > 0.05f ? low_period_s : 0.05f;
  lg->last_row_t   = -1e18;

  const char *home = getenv("HOME");
  if (!home || !*home) home = ".";

  time_t    now = time(NULL);
  struct tm lt;
  localtime_r(&now, &lt);

  char dir[480];  // leaves room for the filename in the 512-byte path
  snprintf(dir, sizeof(dir), "%s/bibby/logs/%04d/%02d/%02d",
           home, lt.tm_year + 1900, lt.tm_mon + 1, lt.tm_mday);
  if (mkdir_p(dir) != 0) {
    fprintf(stderr, "csv_logger: cannot create %s\n", dir);
    return -1;
  }

  snprintf(lg->path, sizeof(lg->path), "%s/%02d-%02d-%02d.csv",
           dir, lt.tm_hour, lt.tm_min, lt.tm_sec);
  lg->f = fopen(lg->path, "w");
  if (!lg->f) {
    fprintf(stderr, "csv_logger: cannot open %s\n", lg->path);
    lg->path[0] = '\0';
    return -1;
  }

  fprintf(lg->f,
          "wall_time,t_monotonic_s,temp_raw_c,temp_filt_c,setpoint_c,"
          "p_demand_w,p_delivered_w,duty1,duty2,flux1_w_cm2,flux2_w_cm2,"
          "pid_ff_w,pid_p_w,pid_i_w,pid_d_w,pid_integral,pid_deriv,"
          "pid_error_c,mc_est_j_per_c,manual,grain_in,rtd_fault,watchdog\n");
  fflush(lg->f);
  return 0;
}

void csv_logger_close(CsvLogger *lg) {
  if (lg->f) fclose(lg->f);
  lg->f = NULL;
}

void csv_logger_log(CsvLogger *lg, const LogRow *row) {
  if (!lg->f) return;
  if (!lg->high_rate && row->t_monotonic_s - lg->last_row_t < lg->low_period_s)
    return;
  lg->last_row_t = row->t_monotonic_s;

  // Wall clock with millisecond resolution, ISO-8601 local time.
  struct timespec ts;
  clock_gettime(CLOCK_REALTIME, &ts);
  struct tm lt;
  localtime_r(&ts.tv_sec, &lt);
  char wall[40];
  size_t k = strftime(wall, sizeof(wall), "%Y-%m-%dT%H:%M:%S", &lt);
  snprintf(wall + k, sizeof(wall) - k, ".%03ld", ts.tv_nsec / 1000000L);

  const PidTerms *p = &row->pid;
  fprintf(lg->f,
          "%s,%.3f,%.6g,%.6g,%.6g,%.6g,%.6g,%.6g,%.6g,%.6g,%.6g,"
          "%.6g,%.6g,%.6g,%.6g,%.6g,%.6g,%.6g,%.6g,%d,%d,%u,%d\n",
          wall, row->t_monotonic_s, row->temp_raw_c, row->temp_filt_c,
          row->setpoint_c, row->p_demand_w, row->p_delivered_w,
          row->duty1, row->duty2, row->flux1_w_cm2, row->flux2_w_cm2,
          p->ff_w, p->p_w, p->i_w, p->d_w, p->integral, p->deriv,
          p->error_c, row->mc_est_j_per_c,
          row->manual ? 1 : 0, row->grain_in ? 1 : 0,
          (unsigned)row->rtd_fault, row->watchdog ? 1 : 0);
  fflush(lg->f);
}

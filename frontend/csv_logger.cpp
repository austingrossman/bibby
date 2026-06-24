#include "csv_logger.h"

#include <cerrno>
#include <cstdlib>
#include <cstring>
#include <ctime>
#include <sys/stat.h>
#include <sys/types.h>

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

CsvLogger::CsvLogger() {
  const char *home = getenv("HOME");
  if (!home || !*home) home = ".";

  time_t     now = time(nullptr);
  struct tm  lt;
  localtime_r(&now, &lt);

  char dir[512];
  snprintf(dir, sizeof(dir), "%s/bibby/logs/%04d/%02d/%02d",
           home, lt.tm_year + 1900, lt.tm_mon + 1, lt.tm_mday);
  if (mkdir_p(dir) != 0) {
    fprintf(stderr, "csv_logger: cannot create %s\n", dir);
    return;
  }

  snprintf(path_, sizeof(path_), "%s/%02d-%02d-%02d.csv",
           dir, lt.tm_hour, lt.tm_min, lt.tm_sec);
  f_ = fopen(path_, "w");
  if (!f_) {
    fprintf(stderr, "csv_logger: cannot open %s\n", path_);
    path_[0] = '\0';
    return;
  }

  fprintf(f_,
          "wall_time,t_monotonic_s,temp_raw_c,temp_filt_c,setpoint_c,"
          "duty1,duty2,pid_output,pid_ff,pid_error,pid_p,pid_i,pid_d,"
          "pid_integral,pid_deriv,manual,grain_in,rtd_fault,watchdog\n");
  fflush(f_);
}

CsvLogger::~CsvLogger() {
  if (f_) fclose(f_);
}

void CsvLogger::log(const LogSample &s) {
  if (!f_) return;

  // Wall clock with millisecond resolution, ISO-8601 local time.
  struct timespec ts;
  clock_gettime(CLOCK_REALTIME, &ts);
  struct tm lt;
  localtime_r(&ts.tv_sec, &lt);
  char wall[40];
  size_t k = strftime(wall, sizeof(wall), "%Y-%m-%dT%H:%M:%S", &lt);
  snprintf(wall + k, sizeof(wall) - k, ".%03ld", ts.tv_nsec / 1000000L);

  const PidTerms &p = s.pid;
  fprintf(f_,
          "%s,%.3f,%.6g,%.6g,%.6g,%.6g,%.6g,%.6g,%.6g,%.6g,%.6g,%.6g,%.6g,"
          "%.6g,%.6g,%d,%d,%u,%d\n",
          wall, s.t_monotonic_s, s.temp_raw_c, s.temp_filt_c, s.setpoint_c,
          s.duty1, s.duty2, p.output, p.ff, p.error, p.p, p.i, p.d,
          p.integral, p.deriv, s.manual ? 1 : 0, s.grain_in ? 1 : 0,
          (unsigned)s.rtd_fault, s.watchdog ? 1 : 0);
  fflush(f_);
}

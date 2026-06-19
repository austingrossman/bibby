#pragma once
#include <cstdint>
#include <cstdio>
#include "pid.h"

// One logged row. The logger fills in the wall-clock timestamp itself at
// log() time; everything else is the controller state for this sample.
struct LogSample {
  double   t_monotonic_s; // SDL monotonic clock, seconds (uniform-ish time base)
  float    temp_raw_c;    // unfiltered sensor reading
  float    temp_filt_c;   // smoothed temperature fed to the PID
  float    setpoint_c;    // target temperature
  float    duty1;         // commanded SSR1 duty [0, 1]
  float    duty2;         // commanded SSR2 duty [0, 1]
  PidTerms pid;           // residual + P/I/D contributions and state
  bool     manual;        // true: sliders drive SSRs; false: PID drives them
  bool     grain_in;      // grain-in toggle state
  uint8_t  rtd_fault;     // MAX31865 fault register (0 = none)
  bool     watchdog;      // heater-controller watchdog alarm
};

// Append-only CSV logger for a single run. Opens a timestamped file under
// ~/bibby/logs/YYYY/MM/DD/ on construction and writes the header; one row per
// log() call, flushed each time so a crash mid-brew keeps the data on disk.
class CsvLogger {
public:
  CsvLogger();
  ~CsvLogger();

  // True if the file opened and the header was written.
  bool ok() const { return f_ != nullptr; }

  // Path of the open log file (empty if !ok()).
  const char *path() const { return path_; }

  void log(const LogSample &s);

private:
  FILE *f_ = nullptr;
  char  path_[512] = {0};
};

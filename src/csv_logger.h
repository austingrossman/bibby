#pragma once
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>

#include "control.h"

// One logged row. The logger fills in the wall-clock timestamp at log() time;
// everything else is the control state for this sample, in engineering units.
typedef struct {
  double   t_monotonic_s;
  float    temp_raw_c;
  float    temp_filt_c;
  float    setpoint_c;
  float    p_demand_w;      // control-law output before the split
  float    p_delivered_w;   // measured from fired half-cycles
  float    duty1, duty2;
  float    flux1_w_cm2, flux2_w_cm2;
  PidTerms pid;
  float    mc_est_j_per_c;  // online thermal-mass estimate (0 = none)
  bool     manual;
  bool     grain_in;
  uint8_t  rtd_fault;
  bool     watchdog;
} LogRow;

// Append-only CSV logger, one timestamped file per run under
// ~/bibby/logs/YYYY/MM/DD/HH-MM-SS.csv (directories auto-created). Every row
// is fflush'ed so a crash mid-brew keeps the data on disk.
//
// Two rates, same schema: high logs every call (one row per fresh sample,
// ~50/60 Hz, for tuning); low decimates to one row per low_period_s seconds
// (for normal brews). Selected by [logging] in bibby.ini.
typedef struct {
  FILE  *f;
  char   path[512];
  bool   high_rate;
  double low_period_s;
  double last_row_t;
} CsvLogger;

// Returns 0 on success (file open, header written), -1 otherwise.
int csv_logger_open(CsvLogger *lg, bool high_rate, float low_period_s);

void csv_logger_close(CsvLogger *lg);

// Write one row, subject to the rate decimation. Safe to call when open failed.
void csv_logger_log(CsvLogger *lg, const LogRow *row);

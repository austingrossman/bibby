#pragma once
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

struct gpiod_line_request;

// MAX31865 RTD-to-digital converter driver. PT100, 3-wire, continuous
// (auto-conversion) mode: a fresh conversion lands every mains-notch period —
// ~60 Hz (16.7 ms) with the 60 Hz filter, ~50 Hz with the 50 Hz filter — and
// DRDY (active low) signals it. Reads are non-blocking: the caller blocks on
// the DRDY edge (or polls) and then calls max31865_read().
typedef struct {
  int      fd;               // spidev file descriptor
  float    ref_resistor;     // Rref, ice-point trimmed (bibby.ini [sensor])
  float    rtd_nominal;      // 100.0 for PT100
  float    cal_gain;         // two-point span trim on the CVD output
  float    cal_offset;
  uint8_t  cfg_reg;          // shadow of the config register
  struct gpiod_line_request *drdy_req;
  unsigned drdy_gpio;
  float    last_temp_c;
  uint8_t  fault_status;     // latched Fault Status register from the last read
} Max31865;

// Datasheet-mandated init: 3-wire + fault-clear + mains notch (must be chosen
// before auto-conversion) -> VBIAS on, 10 ms -> auto-conversion -> discard the
// first conversions. mains_hz selects the notch (50 or 60). Returns 0 on
// success, -1 if the SPI device cannot be opened.
int max31865_init(Max31865 *m, const char *spidev_path, float ref_resistor,
                  struct gpiod_line_request *drdy_req, unsigned drdy_gpio,
                  int mains_hz, float cal_gain, float cal_offset);

void max31865_close(Max31865 *m);

// Non-blocking read. If DRDY is not asserted, or the conversion is flagged
// faulty, returns the previous value with *fresh = false (the filter is never
// poisoned with bad data). Otherwise converts via Callendar-Van Dusen, applies
// the per-unit calibration, and returns the new degC with *fresh = true.
// Every call refreshes fault_status from the latched Fault Status register.
float max31865_read(Max31865 *m, bool *fresh);

static inline uint8_t max31865_fault(const Max31865 *m) { return m->fault_status; }

// Human-readable "; "-separated list of active fault bits into buf.
// Empty string when fs == 0. Bits per MAX31865 datasheet Table 7.
void max31865_fault_text(uint8_t fs, char *buf, size_t buf_size);

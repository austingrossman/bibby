#pragma once
#include <cstdint>
#include <string>

struct gpiod_line_request;

class Max31865 {
public:
  // mains_hz selects the sensor's noise-rejection filter notch (50 or 60 Hz);
  // it must match the local AC line frequency to reject mains pickup.
  // ref_resistor / temp_cal_gain / temp_cal_offset are the per-unit RTD
  // calibration constants loaded from bibby.ini (identity defaults leave the
  // sensor uncalibrated); see README §9 for how each is derived.
  explicit Max31865(const std::string &spidev_path,
                    float ref_resistor = 400.0f,
                    gpiod_line_request *drdy_req = nullptr,
                    int mains_hz = 60,
                    float temp_cal_gain = 1.0f,
                    float temp_cal_offset = 0.0f);
  // Returns °C. If is_fresh is non-null, *is_fresh is set to true when a new,
  // valid conversion was read this call, or false when DRDY was not ready or
  // the conversion was flagged faulty — in both cases the previous (stale)
  // value is returned so the caller's filter is not poisoned.
  float read_temperature(bool *is_fresh = nullptr);

  // Latched Fault Status register (07h) captured by the last read_temperature()
  // burst. 0 means no fault. See decode_fault_status() for the bit meanings.
  uint8_t fault_status() const { return last_fault_status_; }

  // Human-readable, newline-separated list of the active faults in a Fault
  // Status register value (empty string if none). Bits per MAX31865 Table 7.
  static std::string decode_fault_status(uint8_t fs);

  // Write the Fault Status Clear bit to unlatch all fault bits.
  void clear_faults();

private:
  int                  fd_;
  float                ref_resistor_;
  float                rtd_nominal_ = 100.0f; // PT100
  // Per-unit two-point probe-gain calibration applied to the CVD output, loaded
  // from bibby.ini (see README §9). Identity defaults = uncalibrated.
  float                temp_cal_gain_;
  float                temp_cal_offset_;
  uint8_t              cfg_reg_;
  gpiod_line_request  *drdy_req_;
  float                last_temp_read_;
  uint8_t              last_fault_status_ = 0;
};

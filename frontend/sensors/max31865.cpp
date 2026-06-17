#include "max31865.h"
#include <cmath>
#include <cstdint>
#include <fcntl.h>
#include <unistd.h>
#include <sys/ioctl.h>
#include <linux/spi/spidev.h>
#include <stdexcept>
#include <string>
#include <gpiod.h>

static constexpr uint8_t REG_CONFIG            = 0x00;
static constexpr uint8_t REG_RTD_MSB           = 0x01;
static constexpr uint8_t REG_FAULT_STATUS      = 0x07;

static constexpr uint8_t CONFIG_VBIAS_ON       = 0x80;
static constexpr uint8_t CONFIG_CONVERT_AUTO   = 0x40;
static constexpr uint8_t CONFIG_1SHOT          = 0x20;
static constexpr uint8_t CONFIG_3WIRE          = 0x10;
static constexpr uint8_t CONFIG_FAULT_CLEAR    = 0x02;
static constexpr uint8_t CONFIG_50_HZ_FILTER   = 0x01;

// RTD LSB register (02h) bit 0: set when any fault status bit is latched.
static constexpr uint8_t RTD_LSB_FAULT_BIT     = 0x01;

// Fault Status register (07h) bits — MAX31865 Table 7.
static constexpr uint8_t FAULT_RTD_HIGH        = 0x80; // D7: RRTD >= High Fault Threshold
static constexpr uint8_t FAULT_RTD_LOW         = 0x40; // D6: RRTD <= Low Fault Threshold
static constexpr uint8_t FAULT_REFIN_HIGH      = 0x20; // D5: REFIN- > 0.85 x VBIAS
static constexpr uint8_t FAULT_REFIN_LOW       = 0x10; // D4: REFIN- < 0.85 x VBIAS (FORCE- open)
static constexpr uint8_t FAULT_RTDIN_LOW       = 0x08; // D3: RTDIN- < 0.85 x VBIAS (FORCE- open)
static constexpr uint8_t FAULT_OV_UV           = 0x04; // D2: overvoltage / undervoltage

#define NUM_INIT_SAMPLES_TO_THROW_AWAY 2

static void spi_transfer(int fd, const uint8_t *tx, uint8_t *rx, size_t len) {
  struct spi_ioc_transfer tr{};
  tr.tx_buf        = (unsigned long)tx;
  tr.rx_buf        = (unsigned long)rx;
  tr.len           = len;
  tr.speed_hz      = 1000000;
  tr.bits_per_word = 8;
  int ret = ioctl(fd, SPI_IOC_MESSAGE(1), &tr);
  if (ret < 0) {
    // throw std::runtime_error("SPI transfer failed");
    fprintf(stderr, "spi ioctl ret=%d errno=%d\n", ret, errno);
  }
}

Max31865::Max31865(const std::string &path, int ref_resistor, gpiod_line_request *drdy_req)
  : ref_resistor_(ref_resistor), drdy_req_(drdy_req) {

  last_temp_read_ = 0;

  cfg_reg_ = 0;
  uint8_t tx[3]{};
  uint8_t rx[3]{};

  fd_ = open(path.c_str(), O_RDWR);
  if (fd_ < 0) {
    throw std::runtime_error("Cannot open " + path);
  }

  uint8_t mode = SPI_MODE_1;
  ioctl(fd_, SPI_IOC_WR_MODE, &mode);

  // Enable 60Hz filter, 3-wire, clear fault status, normally off
  cfg_reg_ |= CONFIG_3WIRE | CONFIG_FAULT_CLEAR;
  tx[0] = REG_CONFIG | 0x80;
  tx[1] =  cfg_reg_;
  spi_transfer(fd_, tx, rx, 2);

  usleep(1000); // 1ms wait

  // Enable vBias
  cfg_reg_ |= CONFIG_VBIAS_ON;
  tx[0] = REG_CONFIG | 0x80; tx[1] = cfg_reg_;
  spi_transfer(fd_, tx, rx, 2);

  usleep(10000); // 10ms wait for BIAS to come up

  // Enable auto conversion
  cfg_reg_ |= CONFIG_CONVERT_AUTO;
  tx[0] = REG_CONFIG | 0x80; tx[1] = cfg_reg_;
  spi_transfer(fd_, tx, rx, 2);

  usleep(20000); // delay for 20ms, more than enough for a conversion
  
  // Throw away the first conversions
  for (int kk = 0; kk < NUM_INIT_SAMPLES_TO_THROW_AWAY; kk++) {
    gpiod_line_value drdy_value = gpiod_line_request_get_value(drdy_req, 16);
    while (drdy_value == GPIOD_LINE_VALUE_ACTIVE) { // DRDY still high, so now new temp value yet
      usleep(1000); // delay for 1ms
      drdy_value = gpiod_line_request_get_value(drdy_req, 16);
    }
    tx[0] = REG_RTD_MSB; tx[1] = 0; tx[2] = 0;
    spi_transfer(fd_, tx, rx, 3);
    usleep(10000); // delay for 10ms
  }
}

float Max31865::read_temperature(bool *is_fresh) {

  gpiod_line_value val = gpiod_line_request_get_value(drdy_req_, 16);
  if (val == GPIOD_LINE_VALUE_ACTIVE) { // this means data isn't ready yet, just return the last value
    if (is_fresh) *is_fresh = false;
    return last_temp_read_;
  }

  // Burst-read RTD MSB/LSB (01h/02h), the High/Low fault thresholds (03h–06h),
  // and the Fault Status register (07h) in one transaction. The read address
  // auto-increments, so registers 01h..07h land in rx[1..7]. We only act on the
  // RTD data and the fault status, but reading the whole block is a single SPI
  // round trip and guarantees the latched fault status is current.
  uint8_t tx[8]{};
  uint8_t rx[8]{};
  tx[0] = REG_RTD_MSB; // MSB clear = read
  spi_transfer(fd_, tx, rx, 8);

  uint16_t raw = ((uint16_t)rx[1] << 8) | rx[2];
  last_fault_status_ = rx[7];

  // RTD LSB bit 0 is the fault flag: when set, the conversion data is invalid
  // (full-scale or near-zero), so reject the sample, leave the temperature
  // unchanged, and clear the latch so the status reflects the next conversion
  // (a persistent fault simply re-latches).
  if (raw & RTD_LSB_FAULT_BIT) {
    clear_faults();
    if (is_fresh) *is_fresh = false;
    return last_temp_read_;
  }

  raw >>= 1;
  float rtd = ((float)raw / 32768.0f) * (float)ref_resistor_;

  // Callendar-Van Dusen approximation (valid -200–850 °C)
  static constexpr float A = 3.9083e-3f;
  static constexpr float B = -5.775e-7f;
  float r0   = rtd_nominal_;
  float temp = (-A + sqrtf(A * A - 4.0f * B * (1.0f - rtd / r0))) / (2.0f * B);
  last_temp_read_ = temp;
  if (is_fresh) *is_fresh = true;
  return last_temp_read_;
}

void Max31865::clear_faults() {
  // Write the Fault Status Clear bit (D1) with D5/D3/D2 held at 0; it
  // self-clears in hardware. cfg_reg_ already has those bits clear.
  uint8_t tx[2] = { (uint8_t)(REG_CONFIG | 0x80),
                    (uint8_t)(cfg_reg_ | CONFIG_FAULT_CLEAR) };
  uint8_t rx[2]{};
  spi_transfer(fd_, tx, rx, 2);
}

std::string Max31865::decode_fault_status(uint8_t fs) {
  static const struct { uint8_t mask; const char *label; } kBits[] = {
    { FAULT_RTD_HIGH,   "RTD High threshold (open element)" },
    { FAULT_RTD_LOW,    "RTD Low threshold (short)" },
    { FAULT_REFIN_HIGH, "REFIN- > 0.85 x VBIAS" },
    { FAULT_REFIN_LOW,  "REFIN- < 0.85 x VBIAS (FORCE- open)" },
    { FAULT_RTDIN_LOW,  "RTDIN- < 0.85 x VBIAS (FORCE- open)" },
    { FAULT_OV_UV,      "Over/under-voltage" },
  };
  std::string out;
  for (const auto &b : kBits) {
    if (fs & b.mask) {
      if (!out.empty()) out += '\n';
      out += b.label;
    }
  }
  return out;
}

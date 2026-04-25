#include "max31865.h"
#include <cmath>
#include <cstdint>
#include <fcntl.h>
#include <unistd.h>
#include <sys/ioctl.h>
#include <linux/spi/spidev.h>
#include <stdexcept>

static constexpr uint8_t REG_CONFIG      = 0x00;
static constexpr uint8_t REG_RTD_MSB     = 0x01;
static constexpr uint8_t CONFIG_VBIAS_ON = 0x80;
static constexpr uint8_t CONFIG_1SHOT    = 0x20;

static void spi_transfer(int fd, const uint8_t *tx, uint8_t *rx, size_t len) {
  struct spi_ioc_transfer tr{};
  tr.tx_buf        = (unsigned long)tx;
  tr.rx_buf        = (unsigned long)rx;
  tr.len           = len;
  tr.speed_hz      = 1000000;
  tr.bits_per_word = 8;
  if (ioctl(fd, SPI_IOC_MESSAGE(1), &tr) < 0) {
    throw std::runtime_error("SPI transfer failed");
  }
}

Max31865::Max31865(const std::string &path, int ref_resistor)
  : ref_resistor_(ref_resistor) {
  fd_ = open(path.c_str(), O_RDWR);
  if (fd_ < 0) {
    throw std::runtime_error("Cannot open " + path);
  }

  uint8_t mode = SPI_MODE_1;
  ioctl(fd_, SPI_IOC_WR_MODE, &mode);

  // Enable Vbias, 50Hz filter, 2/4-wire
  uint8_t tx[2] = { REG_CONFIG | 0x80, CONFIG_VBIAS_ON | 0x04 };
  uint8_t rx[2]{};
  spi_transfer(fd_, tx, rx, 2);
}

float Max31865::read_temperature() {
  // Trigger one-shot conversion
  uint8_t tx[2] = { REG_CONFIG | 0x80, CONFIG_VBIAS_ON | CONFIG_1SHOT | 0x04 };
  uint8_t rx[2]{};
  spi_transfer(fd_, tx, rx, 2);

  usleep(65000); // 65 ms for conversion

  // Read RTD registers (MSB + LSB)
  uint8_t rtx[3] = { REG_RTD_MSB, 0, 0 };
  uint8_t rrx[3]{};
  spi_transfer(fd_, rtx, rrx, 3);

  uint16_t raw = ((uint16_t)rrx[1] << 8) | rrx[2];
  if (raw & 0x01) {
    return NAN; // fault bit set
  }

  raw >>= 1;
  float rtd = ((float)raw / 32768.0f) * (float)ref_resistor_;

  // Callendar-Van Dusen approximation (valid -200–850 °C)
  static constexpr float A = 3.9083e-3f;
  static constexpr float B = -5.775e-7f;
  float r0   = rtd_nominal_;
  float temp = (-A + sqrtf(A * A - 4.0f * B * (1.0f - rtd / r0))) / (2.0f * B);
  return temp;
}

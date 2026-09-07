#include "max31865.h"

#include <errno.h>
#include <fcntl.h>
#include <gpiod.h>
#include <linux/spi/spidev.h>
#include <math.h>
#include <stdio.h>
#include <string.h>
#include <sys/ioctl.h>
#include <unistd.h>

#define REG_CONFIG           0x00
#define REG_RTD_MSB          0x01
#define REG_FAULT_STATUS     0x07

#define CONFIG_VBIAS_ON      0x80
#define CONFIG_CONVERT_AUTO  0x40
#define CONFIG_3WIRE         0x10
#define CONFIG_FAULT_CLEAR   0x02
#define CONFIG_50HZ_FILTER   0x01

// RTD LSB register (02h) bit 0: set when any fault status bit is latched.
#define RTD_LSB_FAULT_BIT    0x01

#define INIT_SAMPLES_TO_DISCARD 10

static void spi_transfer(int fd, const uint8_t *tx, uint8_t *rx, size_t len) {
  struct spi_ioc_transfer tr = {0};
  tr.tx_buf        = (unsigned long)tx;
  tr.rx_buf        = (unsigned long)rx;
  tr.len           = len;
  tr.speed_hz      = 1000000;
  tr.bits_per_word = 8;
  if (ioctl(fd, SPI_IOC_MESSAGE(1), &tr) < 0)
    fprintf(stderr, "max31865: spi ioctl failed, errno=%d\n", errno);
}

static void write_config(Max31865 *m, uint8_t value) {
  uint8_t tx[2] = { REG_CONFIG | 0x80, value };
  uint8_t rx[2];
  spi_transfer(m->fd, tx, rx, 2);
}

static bool drdy_asserted(const Max31865 *m) {
  // Active low: a low level means a conversion is waiting.
  return gpiod_line_request_get_value(m->drdy_req, m->drdy_gpio) ==
         GPIOD_LINE_VALUE_INACTIVE;
}

int max31865_init(Max31865 *m, const char *spidev_path, float ref_resistor,
                  struct gpiod_line_request *drdy_req, unsigned drdy_gpio,
                  int mains_hz, float cal_gain, float cal_offset) {
  memset(m, 0, sizeof(*m));
  m->ref_resistor = ref_resistor;
  m->rtd_nominal  = 100.0f;
  m->cal_gain     = cal_gain;
  m->cal_offset   = cal_offset;
  m->drdy_req     = drdy_req;
  m->drdy_gpio    = drdy_gpio;

  m->fd = open(spidev_path, O_RDWR);
  if (m->fd < 0) {
    fprintf(stderr, "max31865: cannot open %s\n", spidev_path);
    return -1;
  }
  uint8_t mode = SPI_MODE_1;
  ioctl(m->fd, SPI_IOC_WR_MODE, &mode);

  // The notch bit must be chosen before auto-conversion is enabled (datasheet
  // forbids changing it during auto-conversion); it persists in cfg_reg.
  uint8_t filter_bit = (mains_hz == 50) ? CONFIG_50HZ_FILTER : 0;

  m->cfg_reg = CONFIG_3WIRE | filter_bit;
  write_config(m, m->cfg_reg | CONFIG_FAULT_CLEAR);
  usleep(1000);

  m->cfg_reg |= CONFIG_VBIAS_ON;
  write_config(m, m->cfg_reg);
  usleep(10000);  // VBIAS settling

  m->cfg_reg |= CONFIG_CONVERT_AUTO;
  write_config(m, m->cfg_reg);
  usleep(20000);  // more than one conversion period

  // Discard the first conversions (they settle after VBIAS/auto enable).
  for (int k = 0; k < INIT_SAMPLES_TO_DISCARD; k++) {
    for (int i = 0; i < 100 && !drdy_asserted(m); i++)
      usleep(1000);
    uint8_t tx[3] = { REG_RTD_MSB, 0, 0 };
    uint8_t rx[3];
    spi_transfer(m->fd, tx, rx, 3);
    usleep(10000);
  }
  return 0;
}

void max31865_close(Max31865 *m) {
  if (m->fd >= 0) close(m->fd);
  m->fd = -1;
}

float max31865_read(Max31865 *m, bool *fresh) {
  if (!drdy_asserted(m)) {
    if (fresh) *fresh = false;
    return m->last_temp_c;
  }

  // Burst-read RTD MSB/LSB (01h/02h), the fault thresholds (03h-06h), and the
  // Fault Status register (07h) in one transaction: the read address
  // auto-increments, so registers 01h..07h land in rx[1..7]. One SPI round
  // trip, and the latched fault status is guaranteed current with the sample.
  uint8_t tx[8] = { REG_RTD_MSB };
  uint8_t rx[8] = {0};
  spi_transfer(m->fd, tx, rx, 8);

  uint16_t raw    = ((uint16_t)rx[1] << 8) | rx[2];
  m->fault_status = rx[7];

  // RTD LSB bit 0 is the fault flag: the conversion data is invalid, so hold
  // the previous temperature and clear the latch so the status reflects the
  // next conversion (a persistent fault simply re-latches).
  if (raw & RTD_LSB_FAULT_BIT) {
    write_config(m, m->cfg_reg | CONFIG_FAULT_CLEAR);
    if (fresh) *fresh = false;
    return m->last_temp_c;
  }

  raw >>= 1;
  float rtd = ((float)raw / 32768.0f) * m->ref_resistor;

  // Callendar-Van Dusen approximation (valid over the brewing range).
  const float A = 3.9083e-3f;
  const float B = -5.775e-7f;
  float temp = (-A + sqrtf(A * A - 4.0f * B * (1.0f - rtd / m->rtd_nominal))) /
               (2.0f * B);

  // Per-unit span calibration ([sensor] temp_cal_gain/offset). With Rref
  // already ice-point trimmed, residual probe-alpha error is proportional to
  // T, so it is corrected here in the temperature domain.
  temp = m->cal_gain * temp + m->cal_offset;

  m->last_temp_c = temp;
  if (fresh) *fresh = true;
  return temp;
}

void max31865_fault_text(uint8_t fs, char *buf, size_t buf_size) {
  static const struct { uint8_t mask; const char *label; } bits[] = {
    { 0x80, "RTD high threshold (open element)" },
    { 0x40, "RTD low threshold (short)" },
    { 0x20, "REFIN- > 0.85 x VBIAS" },
    { 0x10, "REFIN- < 0.85 x VBIAS (FORCE- open)" },
    { 0x08, "RTDIN- < 0.85 x VBIAS (FORCE- open)" },
    { 0x04, "over/under-voltage" },
  };
  buf[0] = '\0';
  size_t used = 0;
  for (unsigned i = 0; i < sizeof(bits) / sizeof(bits[0]); i++) {
    if (!(fs & bits[i].mask)) continue;
    int n = snprintf(buf + used, buf_size - used, "%s%s",
                     used ? "; " : "", bits[i].label);
    if (n < 0 || (size_t)n >= buf_size - used) break;
    used += (size_t)n;
  }
}

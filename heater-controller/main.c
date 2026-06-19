#include <stdio.h>
#include <stdlib.h>
#include <stdbool.h>
#include <stdatomic.h>
#include <signal.h>
#include <fcntl.h>
#include <sys/mman.h>
#include <unistd.h>
#include <stdint.h>
#include <gpiod.h>

#include "../shared/shm_types.h"
#include "../shared/single_instance.h"
#include "../shared/config.h"

// GPIO chip and line numbers — adjust for your wiring
#define GPIO_CHIP        "/dev/gpiochip4"   // Pi 5
#define GPIO_ZERO_CROSS  20                 // zero crossing input (rising edge)
#define GPIO_SSR1        21                 // SSR 1 output
#define GPIO_SSR2        26                 // SSR 2 output

// The zero-cross watchdog timeout (also the simulated-ZC period) and the
// frontend-stale ZC count are both derived from the mains frequency in the ini
// file at startup; see config_zc_timeout_ns / config_frontend_watchdog_zc.

static volatile sig_atomic_t running = 1;

static void handle_sigint(int sig) { (void)sig; running = 0; }

static HeaterShm *shm_open_map(void) {
  int fd = shm_open(SHM_NAME, O_CREAT | O_RDWR, 0666);
  if (fd < 0) {
    perror("shm_open");
    exit(1);
  }
  if (ftruncate(fd, sizeof(HeaterShm)) < 0) {
    perror("ftruncate");
    exit(1);
  }
  void *p = mmap(NULL, sizeof(HeaterShm), PROT_READ | PROT_WRITE, MAP_SHARED, fd, 0);
  close(fd);
  if (p == MAP_FAILED) {
    perror("mmap");
    exit(1);
  }
  return (HeaterShm *)p;
}

int main(void) {
  signal(SIGINT, handle_sigint);
  signal(SIGTERM, handle_sigint);

  // Refuse to start if another heater-controller is already running.
  // Lock fd is intentionally left open for the life of the process.
  if (single_instance_lock("/tmp/biab_heater.lock") < 0) {
    fprintf(stderr, "heater-controller: another instance is already running\n");
    return 1;
  }

  BibbyConfig cfg;
  bool cfg_found = config_load(&cfg, NULL);
  uint64_t zc_timeout_ns      = config_zc_timeout_ns(&cfg);
  uint32_t frontend_watchdog  = config_frontend_watchdog_zc(&cfg);
  fprintf(stderr, "heater-controller: %d Hz mains, ZC watchdog %.1f ms%s\n",
          cfg.mains_hz, zc_timeout_ns / 1e6,
          cfg_found ? "" : " (config not found, using defaults)");

  HeaterShm *shm = shm_open_map();
  // The shm segment persists across restarts (never unlinked), so duty1/duty2
  // may hold stale non-zero values from a previous run. Force them to zero at
  // startup so a restart with no frontend running does not immediately fire the
  // SSRs at the old duty cycle (Safety: outputs low on startup).
  atomic_store(&shm->duty1, 0.0f);
  atomic_store(&shm->duty2, 0.0f);
  atomic_store(&shm->output1, false);
  atomic_store(&shm->output2, false);
  atomic_store(&shm->watchdog_alarm, false);
  atomic_store(&shm->simulate_zc, false);
  atomic_store(&shm->iteration, 0);
  atomic_store(&shm->frontend_iteration, 0);

  struct gpiod_chip *chip = gpiod_chip_open(GPIO_CHIP);
  if (!chip) {
    perror("gpiod_chip_open");
    exit(1);
  }

  struct gpiod_request_config *req_cfg = gpiod_request_config_new();
  gpiod_request_config_set_consumer(req_cfg, "heater-controller");

  // Zero crossing — rising edge input
  struct gpiod_line_settings *zc_settings = gpiod_line_settings_new();
  gpiod_line_settings_set_direction(zc_settings, GPIOD_LINE_DIRECTION_INPUT);
  gpiod_line_settings_set_edge_detection(zc_settings, GPIOD_LINE_EDGE_RISING);
  struct gpiod_line_config *zc_cfg = gpiod_line_config_new();
  unsigned int zc_offset = GPIO_ZERO_CROSS;
  gpiod_line_config_add_line_settings(zc_cfg, &zc_offset, 1, zc_settings);
  struct gpiod_line_request *zc_req = gpiod_chip_request_lines(chip, req_cfg, zc_cfg);
  if (!zc_req) {
    perror("request zc");
    exit(1);
  }

  // SSR outputs — initially inactive
  struct gpiod_line_settings *out_settings = gpiod_line_settings_new();
  gpiod_line_settings_set_direction(out_settings, GPIOD_LINE_DIRECTION_OUTPUT);
  gpiod_line_settings_set_output_value(out_settings, GPIOD_LINE_VALUE_INACTIVE);
  struct gpiod_line_config *out_cfg = gpiod_line_config_new();
  unsigned int out_offsets[] = { GPIO_SSR1, GPIO_SSR2 };
  gpiod_line_config_add_line_settings(out_cfg, out_offsets, 2, out_settings);
  struct gpiod_line_request *out_req = gpiod_chip_request_lines(chip, req_cfg, out_cfg);
  if (!out_req) {
    perror("request outputs");
    exit(1);
  }

  gpiod_line_settings_free(zc_settings);
  gpiod_line_config_free(zc_cfg);
  gpiod_line_settings_free(out_settings);
  gpiod_line_config_free(out_cfg);
  gpiod_request_config_free(req_cfg);

  struct gpiod_edge_event_buffer *event_buf = gpiod_edge_event_buffer_new(1);
  if (!event_buf) {
    perror("edge event buffer");
    exit(1);
  }

  float    acc1 = 0.0f, acc2 = 0.0f;
  uint32_t last_frontend_iter = 0;
  uint32_t frontend_stale_zc  = 0;
  bool     frontend_seen      = false;

  while (running) {
    int ret = gpiod_line_request_wait_edge_events(zc_req, zc_timeout_ns);
    if (ret < 0) {
      perror("wait_edge_events");
      break;
    }

    if (ret == 0) {
      if (!atomic_load(&shm->simulate_zc)) {
        atomic_store(&shm->watchdog_alarm, true);
        gpiod_line_request_set_value(out_req, GPIO_SSR1, GPIOD_LINE_VALUE_INACTIVE);
        gpiod_line_request_set_value(out_req, GPIO_SSR2, GPIOD_LINE_VALUE_INACTIVE);
        atomic_store(&shm->output1, false);
        atomic_store(&shm->output2, false);
        continue;
      }
      // simulated ZC — fall through to normal ZC handling
    } else {
      if (gpiod_line_request_read_edge_events(zc_req, event_buf, 1) < 1) {
        continue;
      }
      struct gpiod_edge_event *ev = gpiod_edge_event_buffer_get_event(event_buf, 0);
      if (gpiod_edge_event_get_event_type(ev) != GPIOD_EDGE_EVENT_RISING_EDGE) {
        continue;
      }
    }

    uint32_t iter = atomic_fetch_add(&shm->iteration, 1) + 1;
    printf("iteration: %u\n", iter);
    atomic_store(&shm->watchdog_alarm, false);

    float d1 = atomic_load(&shm->duty1);
    float d2 = atomic_load(&shm->duty2);
    if (d1 < 0.0f) {
      d1 = 0.0f;
    } else if (d1 > 1.0f) {
      d1 = 1.0f;
    }
    if (d2 < 0.0f) {
      d2 = 0.0f;
    } else if (d2 > 1.0f) {
      d2 = 1.0f;
    }

    // Frontend heartbeat watchdog
    uint32_t fe_iter = atomic_load(&shm->frontend_iteration);
    if (fe_iter != last_frontend_iter) {
      last_frontend_iter = fe_iter;
      frontend_stale_zc  = 0;
      frontend_seen      = true;
    } else if (frontend_seen) {
      frontend_stale_zc++;
    }
    if (frontend_seen && frontend_stale_zc >= frontend_watchdog) {
      d1 = 0.0f;
      d2 = 0.0f;
    }

    acc1 += d1;
    bool fire1 = (acc1 >= 1.0f);
    if (fire1) {
      acc1 -= 1.0f;
    }
    if (acc1 > 2.0f) {
        acc1 = 2.0f;
    }
    
    acc2 += d2;
    bool fire2 = (acc2 >= 1.0f);
    if (fire2) {
        acc2 -= 1.0f;
    }
    if (acc2 > 2.0f) {
        acc2 = 2.0f;
    }

    gpiod_line_request_set_value(out_req, GPIO_SSR1, fire1 ? GPIOD_LINE_VALUE_ACTIVE : GPIOD_LINE_VALUE_INACTIVE);
    gpiod_line_request_set_value(out_req, GPIO_SSR2, fire2 ? GPIOD_LINE_VALUE_ACTIVE : GPIOD_LINE_VALUE_INACTIVE);
    atomic_store(&shm->output1, fire1);
    atomic_store(&shm->output2, fire2);
  }

  // Safe shutdown
  gpiod_line_request_set_value(out_req, GPIO_SSR1, GPIOD_LINE_VALUE_INACTIVE);
  gpiod_line_request_set_value(out_req, GPIO_SSR2, GPIOD_LINE_VALUE_INACTIVE);
  atomic_store(&shm->output1, false);
  atomic_store(&shm->output2, false);
  gpiod_edge_event_buffer_free(event_buf);
  gpiod_line_request_release(zc_req);
  gpiod_line_request_release(out_req);
  gpiod_chip_close(chip);
  // Do not shm_unlink here: the frontend may still be mapped to this segment.
  // Unlinking removes the name while the old inode lives on in the frontend's
  // mapping; a restarted heater-controller would then O_CREAT a fresh, separate
  // inode and the two processes would silently stop sharing memory. Leaving the
  // named object in /dev/shm lets a restart reattach to the same segment.
  munmap(shm, sizeof(HeaterShm));
  return 0;
}

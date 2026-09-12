#include <gpiod.h>
#include <pthread.h>
#include <signal.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/mman.h>
#include <time.h>
#include <unistd.h>

#include "config.h"
#include "csv_logger.h"
#include "max31865.h"
#include "sampler_thread.h"
#include "single_instance.h"
#include "ssr_thread.h"
#include "state.h"
#include "ui/ui.h"
#include "web/web.h"

// Pin map (fixed by the HAT; see README).
#define GPIO_CHIP  "/dev/gpiochip4"   // Pi 5
#define GPIO_DRDY  16u                // MAX31865 data-ready, active low
#define GPIO_ZC    20u                // mains zero-cross pulse, rising edge
#define GPIO_SSR1  21u                // element 1 SSR drive
#define GPIO_SSR2  26u                // element 2 SSR drive

static BibbyState g_state;

static void handle_signal(int sig) {
  (void)sig;
  atomic_store(&g_state.running, false);
}

// ── GPIO requests ─────────────────────────────────────────────────────────────
// Each request is created here, before any thread runs; the SSR outputs are
// driven low the instant the request is granted (elements off at startup).
// The kernel releases all lines when the process dies, whatever the cause, and
// the SoC pull-downs then hold the SSR inputs low (elements off on crash).

static struct gpiod_line_request *request_lines(
    struct gpiod_chip *chip, const char *consumer,
    const unsigned *offsets, int count,
    enum gpiod_line_direction direction, enum gpiod_line_edge edge) {
  struct gpiod_line_settings *ls = gpiod_line_settings_new();
  gpiod_line_settings_set_direction(ls, direction);
  if (direction == GPIOD_LINE_DIRECTION_OUTPUT)
    gpiod_line_settings_set_output_value(ls, GPIOD_LINE_VALUE_INACTIVE);
  if (edge != GPIOD_LINE_EDGE_NONE)
    gpiod_line_settings_set_edge_detection(ls, edge);

  struct gpiod_line_config *lc = gpiod_line_config_new();
  gpiod_line_config_add_line_settings(lc, offsets, count, ls);

  struct gpiod_request_config *rc = gpiod_request_config_new();
  gpiod_request_config_set_consumer(rc, consumer);

  struct gpiod_line_request *req = gpiod_chip_request_lines(chip, rc, lc);
  gpiod_request_config_free(rc);
  gpiod_line_config_free(lc);
  gpiod_line_settings_free(ls);
  return req;
}

// Worker threads get explicit modest stacks: the default 8 MB per thread blows
// the RLIMIT_MEMLOCK budget once the process is memory-locked, and these loops
// need a few kilobytes. Returns 0 on success.
static int spawn_thread(pthread_t *tid, void *(*fn)(void *), void *arg,
                        const char *name) {
  pthread_attr_t attr;
  pthread_attr_init(&attr);
  pthread_attr_setstacksize(&attr, 512 * 1024);
  int err = pthread_create(tid, &attr, fn, arg);
  pthread_attr_destroy(&attr);
  if (err != 0) {
    fprintf(stderr, "bibby: cannot start %s thread: %s\n", name, strerror(err));
    atomic_store(&g_state.running, false);
  }
  return err;
}

// ── Headless mode (bench/debug, no display) ───────────────────────────────────

static void headless_loop(BibbyState *st) {
  fprintf(stderr, "bibby: headless mode; ^C to exit\n");
  while (atomic_load(&st->running)) {
    struct timespec ts = { 2, 0 };
    nanosleep(&ts, NULL);
    printf("t=%.1f temp=%.2fC duty=%.2f/%.2f demand=%.0fW delivered=%.0fW "
           "zc=%u wdog=%d fault=0x%02x%s\n",
           bibby_now_s(),
           (double)atomic_load(&st->temp_filt_c),
           (double)atomic_load(&st->duty1), (double)atomic_load(&st->duty2),
           (double)atomic_load(&st->p_demand_w),
           (double)atomic_load(&st->p_delivered_w),
           atomic_load(&st->zc_count),
           atomic_load(&st->watchdog_alarm) ? 1 : 0,
           atomic_load(&st->rtd_fault),
           atomic_load(&st->rtd_unresponsive) ? " (RTD unresponsive)" : "");
    fflush(stdout);
  }
}

// ── main ──────────────────────────────────────────────────────────────────────

int main(int argc, char **argv) {
  const char *config_path = NULL;
  bool  headless = false;
  bool  ui_test  = false;
  bool  sim_zc   = false;
  float manual_w = 0.0f;
  for (int i = 1; i < argc; i++) {
    if (!strcmp(argv[i], "--headless")) headless = true;
    else if (!strcmp(argv[i], "--ui-test")) ui_test = true;  // panel bring-up screen
    else if (!strcmp(argv[i], "--sim-zc")) sim_zc = true;    // bench: no mains
    else if (!strcmp(argv[i], "--manual-w") && i + 1 < argc)
      manual_w = (float)atof(argv[++i]);  // bench/step-test: manual watts at start
    else if (!strcmp(argv[i], "-c") && i + 1 < argc) config_path = argv[++i];
    else {
      fprintf(stderr, "usage: bibby [-c bibby.ini] [--headless] [--ui-test] "
                      "[--sim-zc] [--manual-w W]\n");
      return 1;
    }
  }

  // One instance only; the kernel drops the flock on any exit.
  if (single_instance_lock("/tmp/bibby.lock") < 0) {
    fprintf(stderr, "bibby: another instance is already running\n");
    return 1;
  }

  BibbyConfig cfg;
  bool cfg_found = config_load(&cfg, config_path);
  fprintf(stderr, "bibby: config %s (%s), %d Hz mains, %.0f+%.0f W\n",
          config_resolved_path(), cfg_found ? "loaded" : "NOT FOUND, defaults",
          cfg.mains_hz, (double)cfg.element1_watts, (double)cfg.element2_watts);

  state_init(&g_state);
  atomic_store(&g_state.simulate_zc, sim_zc);
  atomic_store(&g_state.manual_power_w, manual_w);  // manual mode is the default

  struct sigaction sa = {0};
  sa.sa_handler = handle_signal;
  sigaction(SIGINT, &sa, NULL);
  sigaction(SIGTERM, &sa, NULL);

  struct gpiod_chip *chip = gpiod_chip_open(GPIO_CHIP);
  if (!chip) {
    perror("bibby: gpiod_chip_open " GPIO_CHIP);
    return 1;
  }

  // SSR outputs first — granted low, before anything else can run.
  const unsigned ssr_offsets[] = { GPIO_SSR1, GPIO_SSR2 };
  struct gpiod_line_request *ssr_req = request_lines(
      chip, "bibby-ssr", ssr_offsets, 2,
      GPIOD_LINE_DIRECTION_OUTPUT, GPIOD_LINE_EDGE_NONE);
  const unsigned zc_offset = GPIO_ZC;
  struct gpiod_line_request *zc_req = request_lines(
      chip, "bibby-zc", &zc_offset, 1,
      GPIOD_LINE_DIRECTION_INPUT, GPIOD_LINE_EDGE_RISING);
  const unsigned drdy_offset = GPIO_DRDY;
  struct gpiod_line_request *drdy_req = request_lines(
      chip, "bibby-drdy", &drdy_offset, 1,
      GPIOD_LINE_DIRECTION_INPUT, GPIOD_LINE_EDGE_FALLING);
  if (!ssr_req || !zc_req || !drdy_req) {
    perror("bibby: gpiod line request");
    return 1;
  }

  // Sensor init can fail (unwired bench, bad SPI): the controller still runs,
  // but only manual mode is reachable and the UI shows the sensor as faulted.
  Max31865 sensor;
  bool sensor_ok = max31865_init(&sensor, "/dev/spidev0.0",
                                 cfg.sensor_ref_resistor_ohms, drdy_req,
                                 GPIO_DRDY, cfg.mains_hz,
                                 cfg.sensor_temp_cal_gain,
                                 cfg.sensor_temp_cal_offset) == 0;
  if (!sensor_ok)
    fprintf(stderr, "bibby: MAX31865 init failed — manual mode only\n");

  CsvLogger logger;
  if (csv_logger_open(&logger, cfg.log_high_rate, cfg.log_low_period_s) == 0)
    fprintf(stderr, "bibby: logging to %s (%s rate)\n", logger.path,
            cfg.log_high_rate ? "high" : "low");

  SsrThreadArgs ssr_args = {
    .st = &g_state, .cfg = &cfg,
    .zc_req = zc_req, .ssr_req = ssr_req,
    .ssr1_gpio = GPIO_SSR1, .ssr2_gpio = GPIO_SSR2,
  };
  SamplerThreadArgs sampler_args = {
    .st = &g_state, .cfg = &cfg,
    .sensor = sensor_ok ? &sensor : NULL,
    .logger = &logger, .drdy_req = drdy_req,
  };
  pthread_t ssr_tid, sampler_tid;
  if (spawn_thread(&ssr_tid, ssr_thread_main, &ssr_args, "ssr") != 0 ||
      spawn_thread(&sampler_tid, sampler_thread_main, &sampler_args, "sampler") != 0)
    return 1;

  // Lock current mappings (text, data, the thread stacks just created) so the
  // RT thread never page-faults. Deliberately not MCL_FUTURE: the default
  // RLIMIT_MEMLOCK is 8 MB, and locking future UI allocations against that
  // budget makes them fail outright — worse than leaving them pageable.
  if (mlockall(MCL_CURRENT) != 0)
    perror("bibby: mlockall (continuing)");

  // Optional LAN web interface. It is off unless [web] is configured with a
  // password, and it never touches the SSRs directly: remote taps are injected
  // into the UI as pointer events, so control still flows through the panel's
  // own widgets and interlocks. A failure to start is non-fatal.
  web_start(&g_state, &cfg, logger.path);

  if (headless) {
    headless_loop(&g_state);
  } else if (ui_run_mode(&g_state, &cfg, ui_test) != 0) {
    fprintf(stderr, "bibby: UI failed to start; falling back to headless\n");
    headless_loop(&g_state);
  }

  atomic_store(&g_state.running, false);
  web_stop();
  pthread_join(sampler_tid, NULL);
  pthread_join(ssr_tid, NULL);   // drives SSRs low on its way out

  csv_logger_close(&logger);
  if (sensor_ok) max31865_close(&sensor);
  gpiod_line_request_release(drdy_req);
  gpiod_line_request_release(zc_req);
  gpiod_line_request_release(ssr_req);
  gpiod_chip_close(chip);
  return 0;
}

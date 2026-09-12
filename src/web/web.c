#define _GNU_SOURCE
#include "web.h"

#include <arpa/inet.h>
#include <errno.h>
#include <netinet/in.h>
#include <netinet/tcp.h>
#include <poll.h>
#include <pthread.h>
#include <stdatomic.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/socket.h>
#include <unistd.h>

#include "http.h"
#include "web_assets.h"
#include "web_logs.h"
#include "web_screen.h"

#define MAX_CONNECTIONS 16
#define SCREEN_WAIT_S   10.0   // long-poll hold before answering "no change"

static BibbyState       *g_st;
static const BibbyConfig *g_cfg;
static char              g_active_log[512];
static int               g_listen_fd = -1;
static pthread_t         g_accept_tid;
static atomic_bool       g_running;
static atomic_int        g_connections;
static atomic_uint       g_auth_failures;

typedef struct {
  int  fd;
  char peer[INET6_ADDRSTRLEN + 8];
} Conn;

// ── Auth ──────────────────────────────────────────────────────────────────────

static bool authorized(const HttpReq *r) {
  char user[128], pass[256];
  if (!http_basic_auth(r->authorization, user, sizeof(user), pass, sizeof(pass)))
    return false;
  // Both halves are compared length-independently; && would short-circuit on
  // the username and leak which half was wrong.
  bool ok_user = http_secret_equal(user, g_cfg->web_user);
  bool ok_pass = http_secret_equal(pass, g_cfg->web_password);
  return ok_user && ok_pass;
}

static void deny(HttpReq *r, const Conn *c) {
  unsigned n = atomic_fetch_add(&g_auth_failures, 1) + 1;
  if (n <= 20 || n % 50 == 0)
    fprintf(stderr, "web: auth failure #%u from %s\n", n, c->peer);
  // Slow down guessing without tying up the thread for long.
  struct timespec ts = { 0, 300 * 1000 * 1000 };
  nanosleep(&ts, NULL);
  static const char body[] = "authentication required\n";
  http_respond(r, 401, "text/plain; charset=utf-8", body, sizeof(body) - 1,
               "WWW-Authenticate: Basic realm=\"bibby\", charset=\"UTF-8\"\r\n");
}

// ── Endpoints ─────────────────────────────────────────────────────────────────

static void send_json(HttpReq *r, Buf *b) {
  if (b->oom || !b->p) http_respond_text(r, 500, "out of memory");
  else http_respond(r, 200, "application/json", b->p, b->len, NULL);
  buf_free(b);
}

static void route_info(HttpReq *r) {
  int w = 0, h = 0;
  web_screen_size(&w, &h);
  Buf b = {0};
  buf_printf(&b,
             "{\"screen\":%s,\"control\":%s,\"w\":%d,\"h\":%d,"
             "\"mains_hz\":%d,\"max_power_w\":%.0f,\"active_log\":\"%s\"}",
             web_screen_available() ? "true" : "false",
             web_screen_input_ready() ? "true" : "false",
             w, h, g_cfg->mains_hz, (double)config_total_watts(g_cfg),
             g_active_log);
  send_json(r, &b);
}

static void route_state(HttpReq *r) {
  BibbyState *s = g_st;
  Buf b = {0};
  buf_printf(&b,
             "{\"temp_c\":%.2f,\"temp_raw_c\":%.2f,\"temp_valid\":%s,"
             "\"setpoint_c\":%.2f,\"p_demand_w\":%.0f,\"p_delivered_w\":%.0f,"
             "\"duty1\":%.3f,\"duty2\":%.3f,\"manual\":%s,\"grain_in\":%s,"
             "\"rtd_fault\":%u,\"rtd_unresponsive\":%s,\"watchdog\":%s,"
             "\"fault_forced_manual\":%s,\"m_est_l\":%.1f}",
             (double)atomic_load(&s->temp_filt_c),
             (double)atomic_load(&s->temp_raw_c),
             atomic_load(&s->temp_valid) ? "true" : "false",
             (double)atomic_load(&s->setpoint_c),
             (double)atomic_load(&s->p_demand_w),
             (double)atomic_load(&s->p_delivered_w),
             (double)atomic_load(&s->duty1), (double)atomic_load(&s->duty2),
             atomic_load(&s->manual_mode) ? "true" : "false",
             atomic_load(&s->grain_in) ? "true" : "false",
             (unsigned)atomic_load(&s->rtd_fault),
             atomic_load(&s->rtd_unresponsive) ? "true" : "false",
             atomic_load(&s->watchdog_alarm) ? "true" : "false",
             atomic_load(&s->fault_forced_manual) ? "true" : "false",
             (double)atomic_load(&s->m_est_l));
  send_json(r, &b);
}

static void route_screen(HttpReq *r) {
  if (!web_screen_available()) {
    http_respond_text(r, 503, "no display: bibby is running headless");
    return;
  }
  char tmp[32];
  uint32_t since = http_query_get(r, "seq", tmp, sizeof(tmp))
                       ? (uint32_t)strtoul(tmp, NULL, 10) : 0;
  int scale = http_query_get(r, "scale", tmp, sizeof(tmp)) ? atoi(tmp) : 1;

  unsigned char *png = NULL;
  size_t len = 0;
  uint32_t seq = 0;
  if (!web_screen_png(since, scale, SCREEN_WAIT_S, &png, &len, &seq)) {
    http_respond(r, 204, "image/png", NULL, 0, NULL);   // unchanged; poll again
    return;
  }
  char extra[64];
  snprintf(extra, sizeof(extra), "X-Frame-Seq: %u\r\n", seq);
  http_respond(r, 200, "image/png", png, len, extra);
  free(png);
}

static void route_input(HttpReq *r) {
  if (!g_cfg->web_allow_control) {
    http_respond_text(r, 403, "remote control is disabled (web.allow_control)");
    return;
  }
  if (!web_screen_input_ready()) {
    http_respond_text(r, 503, "no display to click on: the UI is not running");
    return;
  }
  int x = 0, y = 0, state = 0;
  if (sscanf(r->body, "%d,%d,%d", &x, &y, &state) != 3) {
    http_respond_text(r, 400, "expected: x,y,state");
    return;
  }
  web_screen_queue_pointer(x, y, state != 0);
  http_respond(r, 204, "text/plain", NULL, 0, NULL);
}

static void route_log_list(HttpReq *r) {
  Buf b = {0};
  web_logs_list_json(&b, g_active_log[0] ? g_active_log : NULL);
  send_json(r, &b);
}

static void route_log_series(HttpReq *r) {
  char path[512], tmp[32];
  if (!http_query_get(r, "path", path, sizeof(path))) {
    http_respond_text(r, 400, "missing ?path=");
    return;
  }
  int points = http_query_get(r, "points", tmp, sizeof(tmp)) ? atoi(tmp) : 1200;
  Buf b = {0};
  if (!web_logs_series_json(&b, path, points)) {
    buf_free(&b);
    http_respond_text(r, 404, "no such log file");
    return;
  }
  send_json(r, &b);
}

static void route_log_download(HttpReq *r) {
  char rel[512], abs[1024];
  if (!http_query_get(r, "path", rel, sizeof(rel)) ||
      !web_logs_resolve(rel, abs, sizeof(abs))) {
    http_respond_text(r, 404, "no such log file");
    return;
  }
  // Flatten "2026/09/07/15-41-11.csv" into one safe download filename.
  char name[sizeof(rel) + 8];
  snprintf(name, sizeof(name), "bibby-%s", rel);
  for (char *p = name; *p; p++)
    if (*p == '/' || *p == '"' || *p == '\\') *p = '-';
  http_respond_file(r, abs, "text/csv", name);
}

static void handle_request(HttpReq *r, const Conn *c) {
  const bool is_get  = !strcmp(r->method, "GET") || !strcmp(r->method, "HEAD");
  const bool is_post = !strcmp(r->method, "POST");
  if (!is_get && !is_post) {
    http_respond_text(r, 405, "method not allowed");
    return;
  }
  if (!authorized(r)) {
    deny(r, c);
    return;
  }

  if (is_get && (!strcmp(r->path, "/") || !strcmp(r->path, "/index.html"))) {
    http_respond(r, 200, "text/html; charset=utf-8", web_index_html,
                 web_index_html_len, NULL);
  } else if (is_get && !strcmp(r->path, "/api/info")) {
    route_info(r);
  } else if (is_get && !strcmp(r->path, "/api/state")) {
    route_state(r);
  } else if (is_get && !strcmp(r->path, "/api/screen.png")) {
    route_screen(r);
  } else if (is_post && !strcmp(r->path, "/api/input")) {
    route_input(r);
  } else if (is_get && !strcmp(r->path, "/api/logs")) {
    route_log_list(r);
  } else if (is_get && !strcmp(r->path, "/api/log")) {
    route_log_series(r);
  } else if (is_get && !strcmp(r->path, "/api/log/download")) {
    route_log_download(r);
  } else {
    http_respond_text(r, 404, "not found");
  }
}

// ── Connection / accept threads ───────────────────────────────────────────────

static void *conn_main(void *arg) {
  Conn *c = arg;
  HttpReq *r = malloc(sizeof(*r));
  if (r) {
    http_req_init(r, c->fd);
    while (atomic_load(&g_running) && http_read_request(r)) {
      handle_request(r, c);
      if (!r->keep_alive) break;
    }
    free(r);
  }
  close(c->fd);
  free(c);
  atomic_fetch_sub(&g_connections, 1);
  return NULL;
}

static void *accept_main(void *arg) {
  (void)arg;
  while (atomic_load(&g_running)) {
    struct pollfd pfd = { .fd = g_listen_fd, .events = POLLIN };
    int pr = poll(&pfd, 1, 200);      // bounded, so shutdown is prompt
    if (pr <= 0) continue;

    struct sockaddr_storage sa;
    socklen_t sl = sizeof(sa);
    int fd = accept(g_listen_fd, (struct sockaddr *)&sa, &sl);
    if (fd < 0) continue;

    if (atomic_load(&g_connections) >= MAX_CONNECTIONS) {
      static const char busy[] =
          "HTTP/1.1 503 Service Unavailable\r\nContent-Length: 0\r\n"
          "Connection: close\r\n\r\n";
      http_send_all(fd, busy, sizeof(busy) - 1);
      close(fd);
      continue;
    }

    // An abandoned browser tab must not hold a thread forever.
    struct timeval tv = { 30, 0 };
    setsockopt(fd, SOL_SOCKET, SO_RCVTIMEO, &tv, sizeof(tv));
    tv.tv_sec = 20;
    setsockopt(fd, SOL_SOCKET, SO_SNDTIMEO, &tv, sizeof(tv));
    int one = 1;
    setsockopt(fd, IPPROTO_TCP, TCP_NODELAY, &one, sizeof(one));

    Conn *c = calloc(1, sizeof(*c));
    if (!c) { close(fd); continue; }
    c->fd = fd;
    if (sa.ss_family == AF_INET) {
      char ip[INET_ADDRSTRLEN] = "?";
      inet_ntop(AF_INET, &((struct sockaddr_in *)&sa)->sin_addr, ip, sizeof(ip));
      snprintf(c->peer, sizeof(c->peer), "%s", ip);
    } else {
      snprintf(c->peer, sizeof(c->peer), "peer");
    }

    pthread_attr_t attr;
    pthread_attr_init(&attr);
    pthread_attr_setstacksize(&attr, 256 * 1024);
    pthread_attr_setdetachstate(&attr, PTHREAD_CREATE_DETACHED);
    pthread_t tid;
    atomic_fetch_add(&g_connections, 1);
    if (pthread_create(&tid, &attr, conn_main, c) != 0) {
      atomic_fetch_sub(&g_connections, 1);
      close(fd);
      free(c);
    }
    pthread_attr_destroy(&attr);
  }
  return NULL;
}

// ── Lifecycle ─────────────────────────────────────────────────────────────────

int web_start(BibbyState *st, const BibbyConfig *cfg, const char *active_log_path) {
  if (!cfg->web_enable) return -1;
  if (!cfg->web_password[0]) {
    fprintf(stderr, "web: [web] enable is set but web.password is empty — "
                    "refusing to serve the panel without one\n");
    return -1;
  }
  g_st  = st;
  g_cfg = cfg;
  snprintf(g_active_log, sizeof(g_active_log), "%s",
           active_log_path ? active_log_path : "");

  g_listen_fd = socket(AF_INET, SOCK_STREAM, 0);
  if (g_listen_fd < 0) {
    perror("web: socket");
    return -1;
  }
  int one = 1;
  setsockopt(g_listen_fd, SOL_SOCKET, SO_REUSEADDR, &one, sizeof(one));

  struct sockaddr_in addr = {0};
  addr.sin_family = AF_INET;
  addr.sin_port   = htons((uint16_t)cfg->web_port);
  if (inet_pton(AF_INET, cfg->web_bind, &addr.sin_addr) != 1)
    addr.sin_addr.s_addr = htonl(INADDR_ANY);

  if (bind(g_listen_fd, (struct sockaddr *)&addr, sizeof(addr)) != 0 ||
      listen(g_listen_fd, 16) != 0) {
    fprintf(stderr, "web: cannot listen on %s:%d: %s\n", cfg->web_bind,
            cfg->web_port, strerror(errno));
    close(g_listen_fd);
    g_listen_fd = -1;
    return -1;
  }

  // The mirror is optional: without a framebuffer the log browser still works.
  const char *fb = strcmp(cfg->ui_fb_device, "auto") ? cfg->ui_fb_device : "/dev/fb0";
  web_screen_start(fb, cfg->web_screen_fps, cfg->ui_rotation);

  atomic_store(&g_running, true);
  pthread_attr_t attr;
  pthread_attr_init(&attr);
  pthread_attr_setstacksize(&attr, 256 * 1024);
  int err = pthread_create(&g_accept_tid, &attr, accept_main, NULL);
  pthread_attr_destroy(&attr);
  if (err != 0) {
    fprintf(stderr, "web: cannot start accept thread: %s\n", strerror(err));
    atomic_store(&g_running, false);
    close(g_listen_fd);
    g_listen_fd = -1;
    web_screen_stop();
    return -1;
  }

  fprintf(stderr, "web: http://%s:%d/ as user \"%s\" (%s)\n", cfg->web_bind,
          cfg->web_port, cfg->web_user,
          cfg->web_allow_control ? "remote control ENABLED" : "view only");
  return 0;
}

void web_stop(void) {
  if (g_listen_fd < 0) return;
  atomic_store(&g_running, false);
  pthread_join(g_accept_tid, NULL);
  close(g_listen_fd);
  g_listen_fd = -1;
  web_screen_stop();
  // Detached connection threads exit on their own socket timeouts; the process
  // is on its way out and their sockets close with it.
}

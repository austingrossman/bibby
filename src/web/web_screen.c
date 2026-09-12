#define _GNU_SOURCE
#include "web_screen.h"

#include <errno.h>
#include <fcntl.h>
#include <linux/fb.h>
#include <pthread.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/ioctl.h>
#include <sys/mman.h>
#include <time.h>
#include <unistd.h>
#include <zlib.h>

#include "lvgl.h"

#include "../state.h"

// ── Framebuffer ───────────────────────────────────────────────────────────────

typedef struct {
  int       fd;
  uint8_t  *map;
  size_t    map_len;
  int       w, h;
  size_t    stride;          // bytes per line
  int       bytes_per_px;
  struct { int off, len; } r, g, b;
  size_t    visible_bytes;   // h * stride
  int       rotation;        // ui.rotation: panel -> what the operator sees
  int       dw, dh;          // display size (panel size, rotated)
} Framebuffer;

static Framebuffer fb;
static bool        fb_ok;

typedef struct {
  unsigned char *png;
  size_t         len;
  uint32_t       seq;
} PngCache;

static pthread_mutex_t m   = PTHREAD_MUTEX_INITIALIZER;
static pthread_cond_t  cv  = PTHREAD_COND_INITIALIZER;  // new frame available
static pthread_cond_t  cv_viewers = PTHREAD_COND_INITIALIZER;

static uint8_t  *cur_frame;      // last captured copy of the visible fb
static uint32_t  frame_seq;      // bumped whenever cur_frame changes
static int       viewers;        // clients currently waiting for frames
static PngCache  cache[2];       // [0] = scale 1, [1] = scale 2
static uint8_t  *scratch_filt;   // PNG filtered scanlines (under `m`)
static uint8_t  *scratch_z;      // deflate output (under `m`)
static size_t    scratch_filt_cap, scratch_z_cap;

static pthread_t capture_tid;
static bool      capture_running;
static int       capture_fps = 5;

static int fb_open(const char *path, int rotation) {
  memset(&fb, 0, sizeof(fb));
  fb.fd = open(path, O_RDONLY);
  if (fb.fd < 0) {
    fprintf(stderr, "web: cannot open %s: %s\n", path, strerror(errno));
    return -1;
  }
  struct fb_var_screeninfo vi;
  struct fb_fix_screeninfo fi;
  if (ioctl(fb.fd, FBIOGET_VSCREENINFO, &vi) != 0 ||
      ioctl(fb.fd, FBIOGET_FSCREENINFO, &fi) != 0) {
    fprintf(stderr, "web: %s screeninfo: %s\n", path, strerror(errno));
    close(fb.fd);
    return -1;
  }
  if (vi.bits_per_pixel != 16 && vi.bits_per_pixel != 32) {
    fprintf(stderr, "web: unsupported framebuffer depth %u bpp\n",
            vi.bits_per_pixel);
    close(fb.fd);
    return -1;
  }

  fb.w = (int)vi.xres;
  fb.h = (int)vi.yres;
  fb.stride = fi.line_length;
  fb.bytes_per_px = (int)vi.bits_per_pixel / 8;
  fb.r.off = (int)vi.red.offset;    fb.r.len = (int)vi.red.length;
  fb.g.off = (int)vi.green.offset;  fb.g.len = (int)vi.green.length;
  fb.b.off = (int)vi.blue.offset;   fb.b.len = (int)vi.blue.length;
  fb.rotation = (rotation == 90 || rotation == 180 || rotation == 270) ? rotation : 0;
  fb.dw = (fb.rotation == 90 || fb.rotation == 270) ? fb.h : fb.w;
  fb.dh = (fb.rotation == 90 || fb.rotation == 270) ? fb.w : fb.h;
  fb.visible_bytes = (size_t)fb.h * fb.stride;
  fb.map_len = fi.smem_len ? fi.smem_len : fb.visible_bytes;
  if (fb.map_len < fb.visible_bytes) fb.map_len = fb.visible_bytes;

  fb.map = mmap(NULL, fb.map_len, PROT_READ, MAP_SHARED, fb.fd, 0);
  if (fb.map == MAP_FAILED) {
    fprintf(stderr, "web: mmap %s: %s\n", path, strerror(errno));
    close(fb.fd);
    fb.map = NULL;
    return -1;
  }
  fprintf(stderr, "web: mirroring %s, panel %dx%d %d bpp, rotation %d -> %dx%d\n",
          path, fb.w, fb.h, fb.bytes_per_px * 8, fb.rotation, fb.dw, fb.dh);
  return 0;
}

// Expand an `len`-bit channel to 8 bits by bit replication (5 bits: 0x1F -> 0xFF).
static inline uint8_t chan8(uint32_t px, int off, int len) {
  if (len <= 0) return 0;
  uint32_t v = (px >> off) & ((1u << len) - 1u);
  if (len >= 8) return (uint8_t)(v >> (len - 8));
  v <<= (8 - len);
  return (uint8_t)(v | (v >> len));
}

// The panel is mounted sideways and LVGL rotates into it in software, so the
// raw framebuffer is sideways too. Undo that here: the browser gets the image
// the operator actually sees, and pointer coordinates come back through
// panel_from_display() to land on the same pixel a finger would.
//
// The forward map matches LVGL's own indev rotation (indev_pointer_proc), so
// display space here is exactly LVGL's logical screen.
static inline void panel_from_display(int lx, int ly, int *px, int *py) {
  switch (fb.rotation) {
    case 90:  *px = ly;            *py = fb.h - 1 - lx; break;
    case 180: *px = fb.w - 1 - lx; *py = fb.h - 1 - ly; break;
    case 270: *px = fb.w - 1 - ly; *py = lx;            break;
    default:  *px = lx;            *py = ly;            break;
  }
}

// Walking the mirrored image in display order means walking the framebuffer
// along a column whenever the panel is rotated. The coordinate transform above
// is the same for every pixel in a row, so it collapses into two constants: a
// byte step from one output pixel to the next, and one from row to row. Doing
// it this way instead of transforming each pixel is worth about 10x — the
// conversion, not the PNG compression, is what the mirror actually costs.
typedef struct {
  const uint8_t *base;   // source of display pixel (0,0)
  ptrdiff_t      pix;    // bytes from one display pixel to the next in x
  ptrdiff_t      row;    // bytes from one display row to the next in y
} DisplayWalk;

static DisplayWalk display_walk(void) {
  const ptrdiff_t st = (ptrdiff_t)fb.stride, bp = (ptrdiff_t)fb.bytes_per_px;
  const size_t last_row = (size_t)(fb.h - 1) * fb.stride;
  const size_t last_col = (size_t)(fb.w - 1) * (size_t)fb.bytes_per_px;
  DisplayWalk w;
  switch (fb.rotation) {
    case 90:  w.base = cur_frame + last_row;            w.pix = -st; w.row =  bp; break;
    case 180: w.base = cur_frame + last_row + last_col; w.pix = -bp; w.row = -st; break;
    case 270: w.base = cur_frame + last_col;            w.pix =  st; w.row = -bp; break;
    default:  w.base = cur_frame;                       w.pix =  bp; w.row =  st; break;
  }
  return w;
}

// Generic pixel decode for framebuffer layouts the fast path below rejects.
static inline void decode_px(const uint8_t *q, uint32_t *r, uint32_t *g,
                             uint32_t *b) {
  uint32_t v = fb.bytes_per_px == 2
                   ? (uint32_t)(q[0] | (q[1] << 8))
                   : (uint32_t)(q[0] | (q[1] << 8) | (q[2] << 16) |
                                ((uint32_t)q[3] << 24));
  *r = chan8(v, fb.r.off, fb.r.len);
  *g = chan8(v, fb.g.off, fb.g.len);
  *b = chan8(v, fb.b.off, fb.b.len);
}

// ── PNG encoding ─────// ── PNG encoding ──────────────────────────────────────────────────────────────
// Truecolour 8-bit RGB, one IDAT, "Up" filter on every scanline: cheap to
// compute and it turns the flat vertical runs of a UI screenshot into zeros.

static void put_be32(uint8_t *p, uint32_t v) {
  p[0] = (uint8_t)(v >> 24); p[1] = (uint8_t)(v >> 16);
  p[2] = (uint8_t)(v >> 8);  p[3] = (uint8_t)v;
}

static uint8_t *png_chunk(uint8_t *p, const char *type, const uint8_t *data,
                          size_t len) {
  put_be32(p, (uint32_t)len);
  memcpy(p + 4, type, 4);
  if (len) memcpy(p + 8, data, len);
  uint32_t crc = (uint32_t)crc32(0, p + 4, (unsigned)(len + 4));
  put_be32(p + 8 + len, crc);
  return p + 12 + len;
}

static bool scratch_reserve(uint8_t **buf, size_t *cap, size_t need) {
  if (*cap >= need) return true;
  uint8_t *n = realloc(*buf, need);
  if (!n) return false;
  *buf = n;
  *cap = need;
  return true;
}

// Encode cur_frame into cache[idx]. Caller holds `m`.
static bool encode_locked(int idx) {
  const int scale = idx == 1 ? 2 : 1;
  const int ow = fb.dw / scale, oh = fb.dh / scale;
  if (ow <= 0 || oh <= 0) return false;

  const size_t row_bytes = (size_t)ow * 3;
  const size_t raw_len   = (size_t)oh * (row_bytes + 1);
  if (!scratch_reserve(&scratch_filt, &scratch_filt_cap, raw_len)) return false;

  // Un-rotate (and optionally box-downscale) into the filtered buffer, then
  // apply the Up filter in place, walking bottom-up so each row still sees the
  // untouched row above it.
  const DisplayWalk w = display_walk();

  // 32 bpp with byte-aligned 8-bit channels (every Pi framebuffer bibby has
  // met) needs no shifting at all — just pick three bytes out of the pixel.
  const bool byte_channels =
      fb.bytes_per_px == 4 && fb.r.len == 8 && fb.g.len == 8 && fb.b.len == 8 &&
      fb.r.off % 8 == 0 && fb.g.off % 8 == 0 && fb.b.off % 8 == 0;
  const int ri = fb.r.off / 8, gi = fb.g.off / 8, bi = fb.b.off / 8;

  for (int y = 0; y < oh; y++) {
    uint8_t *out = scratch_filt + (size_t)y * (row_bytes + 1);
    out[0] = 2;  // filter type: Up
    uint8_t *px = out + 1;
    const uint8_t *p = w.base + (ptrdiff_t)(y * scale) * w.row;

    if (scale == 1) {
      if (byte_channels) {
        for (int x = 0; x < ow; x++, p += w.pix, px += 3) {
          px[0] = p[ri]; px[1] = p[gi]; px[2] = p[bi];
        }
      } else {
        for (int x = 0; x < ow; x++, p += w.pix, px += 3) {
          uint32_t r, g, b;
          decode_px(p, &r, &g, &b);
          px[0] = (uint8_t)r; px[1] = (uint8_t)g; px[2] = (uint8_t)b;
        }
      }
    } else {
      // 2x2 box average; the four source pixels are one pix/row step apart.
      const ptrdiff_t step = w.pix * 2;
      for (int x = 0; x < ow; x++, p += step, px += 3) {
        const uint8_t *q0 = p, *q1 = p + w.pix;
        const uint8_t *q2 = p + w.row, *q3 = q2 + w.pix;
        if (byte_channels) {
          px[0] = (uint8_t)((q0[ri] + q1[ri] + q2[ri] + q3[ri]) >> 2);
          px[1] = (uint8_t)((q0[gi] + q1[gi] + q2[gi] + q3[gi]) >> 2);
          px[2] = (uint8_t)((q0[bi] + q1[bi] + q2[bi] + q3[bi]) >> 2);
        } else {
          uint32_t ar = 0, ag = 0, ab = 0, r, g, b;
          decode_px(q0, &r, &g, &b); ar += r; ag += g; ab += b;
          decode_px(q1, &r, &g, &b); ar += r; ag += g; ab += b;
          decode_px(q2, &r, &g, &b); ar += r; ag += g; ab += b;
          decode_px(q3, &r, &g, &b); ar += r; ag += g; ab += b;
          px[0] = (uint8_t)(ar >> 2); px[1] = (uint8_t)(ag >> 2); px[2] = (uint8_t)(ab >> 2);
        }
      }
    }
  }
  for (int y = oh - 1; y >= 0; y--) {
    uint8_t *row = scratch_filt + (size_t)y * (row_bytes + 1) + 1;
    if (y == 0) break;  // row 0's "previous row" is all zeros: leave it as-is
    const uint8_t *prev = row - (row_bytes + 1);
    for (size_t i = 0; i < row_bytes; i++) row[i] = (uint8_t)(row[i] - prev[i]);
  }

  uLongf zlen = compressBound((uLong)raw_len);
  if (!scratch_reserve(&scratch_z, &scratch_z_cap, zlen)) return false;
  if (compress2(scratch_z, &zlen, scratch_filt, (uLong)raw_len, 3) != Z_OK)
    return false;

  const size_t total = 8 + 25 + (12 + zlen) + 12;
  unsigned char *png = malloc(total);
  if (!png) return false;

  static const uint8_t sig[8] = { 137, 'P', 'N', 'G', '\r', '\n', 26, '\n' };
  memcpy(png, sig, 8);
  uint8_t ihdr[13];
  put_be32(ihdr + 0, (uint32_t)ow);
  put_be32(ihdr + 4, (uint32_t)oh);
  ihdr[8] = 8;   // bit depth
  ihdr[9] = 2;   // colour type: truecolour
  ihdr[10] = ihdr[11] = ihdr[12] = 0;
  uint8_t *p = png_chunk(png + 8, "IHDR", ihdr, sizeof(ihdr));
  p = png_chunk(p, "IDAT", scratch_z, zlen);
  p = png_chunk(p, "IEND", NULL, 0);

  free(cache[idx].png);
  cache[idx].png = png;
  cache[idx].len = (size_t)(p - png);
  cache[idx].seq = frame_seq;
  return true;
}

// ── Capture thread ────────────────────────────────────────────────────────────

static void *capture_main(void *arg) {
  (void)arg;
  pthread_mutex_lock(&m);
  while (capture_running) {
    while (capture_running && viewers == 0)
      pthread_cond_wait(&cv_viewers, &m);
    if (!capture_running) break;

    // The panning offset moves if the driver ever double-buffers; re-read it
    // rather than assume the visible page starts at byte 0.
    size_t base = 0;
    struct fb_var_screeninfo vi;
    if (ioctl(fb.fd, FBIOGET_VSCREENINFO, &vi) == 0) {
      base = (size_t)vi.yoffset * fb.stride;
      if (base + fb.visible_bytes > fb.map_len) base = 0;
    }
    const uint8_t *live = fb.map + base;
    if (memcmp(cur_frame, live, fb.visible_bytes) != 0) {
      memcpy(cur_frame, live, fb.visible_bytes);
      frame_seq++;
      if (frame_seq == 0) frame_seq = 1;   // 0 means "no frame yet" to clients
      pthread_cond_broadcast(&cv);
    }
    pthread_mutex_unlock(&m);

    struct timespec ts = { 0, 0 };
    long period_ns = 1000000000L / (capture_fps > 0 ? capture_fps : 5);
    ts.tv_sec  = period_ns / 1000000000L;
    ts.tv_nsec = period_ns % 1000000000L;
    nanosleep(&ts, NULL);
    pthread_mutex_lock(&m);
  }
  pthread_mutex_unlock(&m);
  return NULL;
}

int web_screen_start(const char *fb_path, int fps, int rotation) {
  if (fb_open(fb_path, rotation) != 0) return -1;
  cur_frame = calloc(1, fb.visible_bytes);
  if (!cur_frame) {
    munmap(fb.map, fb.map_len);
    close(fb.fd);
    return -1;
  }
  capture_fps = fps < 1 ? 1 : (fps > 30 ? 30 : fps);
  fb_ok = true;
  capture_running = true;

  pthread_attr_t attr;
  pthread_attr_init(&attr);
  pthread_attr_setstacksize(&attr, 256 * 1024);
  int err = pthread_create(&capture_tid, &attr, capture_main, NULL);
  pthread_attr_destroy(&attr);
  if (err != 0) {
    fprintf(stderr, "web: cannot start capture thread: %s\n", strerror(err));
    capture_running = false;
    fb_ok = false;
    return -1;
  }
  return 0;
}

void web_screen_stop(void) {
  // Connection threads are detached and may be mid-request, so retire the
  // mirror under the lock that guards the frame buffers: a request either
  // finishes before the free, or sees fb_ok false and declines.
  pthread_mutex_lock(&m);
  if (!fb_ok) { pthread_mutex_unlock(&m); return; }
  fb_ok = false;
  capture_running = false;
  pthread_cond_broadcast(&cv_viewers);
  pthread_cond_broadcast(&cv);
  pthread_mutex_unlock(&m);
  pthread_join(capture_tid, NULL);

  pthread_mutex_lock(&m);
  free(cache[0].png); free(cache[1].png);
  memset(cache, 0, sizeof(cache));
  free(scratch_filt); free(scratch_z);
  scratch_filt = scratch_z = NULL;
  scratch_filt_cap = scratch_z_cap = 0;
  free(cur_frame);
  cur_frame = NULL;
  pthread_mutex_unlock(&m);

  munmap(fb.map, fb.map_len);
  close(fb.fd);
}

bool web_screen_available(void) { return fb_ok; }

void web_screen_size(int *w, int *h) {
  *w = fb_ok ? fb.dw : 0;
  *h = fb_ok ? fb.dh : 0;
}

bool web_screen_png(uint32_t since_seq, int scale, double timeout_s,
                    unsigned char **png, size_t *len, uint32_t *seq) {
  const int idx = (scale == 2) ? 1 : 0;

  struct timespec deadline;
  clock_gettime(CLOCK_REALTIME, &deadline);
  deadline.tv_sec  += (time_t)timeout_s;
  deadline.tv_nsec += (long)((timeout_s - (double)(time_t)timeout_s) * 1e9);
  if (deadline.tv_nsec >= 1000000000L) { deadline.tv_sec++; deadline.tv_nsec -= 1000000000L; }

  pthread_mutex_lock(&m);
  if (!fb_ok) { pthread_mutex_unlock(&m); return false; }
  viewers++;
  pthread_cond_signal(&cv_viewers);
  while (capture_running && frame_seq == since_seq) {
    if (pthread_cond_timedwait(&cv, &m, &deadline) == ETIMEDOUT) break;
  }

  bool ok = false;
  if (fb_ok && frame_seq != 0 && frame_seq != since_seq) {
    if (cache[idx].seq != frame_seq) encode_locked(idx);
    if (cache[idx].seq == frame_seq && cache[idx].png) {
      unsigned char *copy = malloc(cache[idx].len);
      if (copy) {
        memcpy(copy, cache[idx].png, cache[idx].len);
        *png = copy;
        *len = cache[idx].len;
        *seq = frame_seq;
        ok = true;
      }
    }
  }
  viewers--;
  pthread_mutex_unlock(&m);
  return ok;
}

// ── Pointer injection ─────────────────────────────────────────────────────────
// Web threads push panel-coordinate samples here; the LVGL indev read callback
// on the UI thread drains them. LVGL is single-threaded, so this queue is the
// whole boundary — no lv_* call is ever made from a web thread.

#define PTR_QUEUE 128

typedef struct { int16_t x, y; uint8_t pressed; } PointerSample;

static pthread_mutex_t ptr_m = PTHREAD_MUTEX_INITIALIZER;
static PointerSample   ptr_q[PTR_QUEUE];
static int             ptr_head, ptr_tail;   // head == tail: empty
static bool            indev_ready;

// Held-press timeout. A browser that vanishes mid-drag must not leave a widget
// pressed forever, so a press with no follow-up is released after this long.
#define PTR_HOLD_TIMEOUT_S 2.0

// x/y arrive in display coordinates (what the browser shows); LVGL wants the
// panel coordinates its own rotation step expects, the same ones evdev feeds.
void web_screen_queue_pointer(int x, int y, bool pressed) {
  if (!indev_ready || !fb_ok) return;
  if (x < 0) x = 0;
  if (y < 0) y = 0;
  if (x > fb.dw - 1) x = fb.dw - 1;
  if (y > fb.dh - 1) y = fb.dh - 1;
  int px, py;
  panel_from_display(x, y, &px, &py);
  x = px;
  y = py;
  pthread_mutex_lock(&ptr_m);
  int next = (ptr_head + 1) % PTR_QUEUE;
  if (next == ptr_tail) ptr_tail = (ptr_tail + 1) % PTR_QUEUE;  // drop oldest
  ptr_q[ptr_head].x = (int16_t)x;
  ptr_q[ptr_head].y = (int16_t)y;
  ptr_q[ptr_head].pressed = pressed ? 1 : 0;
  ptr_head = next;
  pthread_mutex_unlock(&ptr_m);
}

static void web_indev_read(lv_indev_t *indev, lv_indev_data_t *data) {
  (void)indev;
  static int16_t last_x, last_y;
  static bool    last_pressed;
  static double  last_event_t;

  pthread_mutex_lock(&ptr_m);
  bool more = false;
  if (ptr_head != ptr_tail) {
    PointerSample s = ptr_q[ptr_tail];
    ptr_tail = (ptr_tail + 1) % PTR_QUEUE;
    last_x = s.x;
    last_y = s.y;
    last_pressed = s.pressed != 0;
    last_event_t = bibby_now_s();
    more = ptr_head != ptr_tail;
  } else if (last_pressed && bibby_now_s() - last_event_t > PTR_HOLD_TIMEOUT_S) {
    last_pressed = false;   // client went away mid-press
  }
  pthread_mutex_unlock(&ptr_m);

  data->point.x = last_x;
  data->point.y = last_y;
  data->state = last_pressed ? LV_INDEV_STATE_PRESSED : LV_INDEV_STATE_RELEASED;
  data->continue_reading = more;
}

bool web_screen_input_ready(void) { return indev_ready; }

void web_screen_attach_indev(void) {
  lv_indev_t *indev = lv_indev_create();
  if (!indev) {
    fprintf(stderr, "web: cannot create pointer indev; remote control disabled\n");
    return;
  }
  lv_indev_set_type(indev, LV_INDEV_TYPE_POINTER);
  lv_indev_set_read_cb(indev, web_indev_read);
  lv_indev_set_display(indev, lv_display_get_default());
  indev_ready = true;
  fprintf(stderr, "web: remote pointer attached\n");
}

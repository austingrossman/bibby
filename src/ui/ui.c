// LVGL backend: fbdev display on the DSI panel (software-rotated for the
// sideways mount), evdev touch. Runs on the main thread; the control and
// safety paths never wait on anything here.
#include "ui.h"

#include <fcntl.h>
#include <linux/input.h>
#include <stdio.h>
#include <string.h>
#include <sys/ioctl.h>
#include <unistd.h>

#include "../web/web_screen.h"
#include "lvgl.h"
#include "ui_screen.h"

static uint32_t tick_ms(void) {
  return (uint32_t)(bibby_now_s() * 1000.0);
}

// First evdev node advertising absolute touch coordinates (the Goodix panel).
// Returns true and fills path on success.
static bool find_touch_device(char *path, size_t path_size) {
  for (int i = 0; i < 32; i++) {
    char p[32];
    snprintf(p, sizeof(p), "/dev/input/event%d", i);
    int fd = open(p, O_RDONLY | O_NONBLOCK);
    if (fd < 0) continue;

    unsigned long abs_bits[(ABS_MAX + 63) / 64] = {0};
    bool touch = ioctl(fd, EVIOCGBIT(EV_ABS, sizeof(abs_bits)), abs_bits) >= 0 &&
                 (abs_bits[ABS_MT_POSITION_X / 64] >> (ABS_MT_POSITION_X % 64)) & 1;
    close(fd);
    if (touch) {
      snprintf(path, path_size, "%s", p);
      return true;
    }
  }
  return false;
}

static lv_display_rotation_t rotation_from_degrees(int deg) {
  switch (deg) {
    case 90:  return LV_DISPLAY_ROTATION_90;
    case 180: return LV_DISPLAY_ROTATION_180;
    case 270: return LV_DISPLAY_ROTATION_270;
    default:  return LV_DISPLAY_ROTATION_0;
  }
}

// ── Bring-up test screen (bibby --ui-test) ────────────────────────────────────
// A labelled border, corner markers, and a crosshair that follows the finger:
// verifies rotation (label upright, "TL" top-left as the enclosure is viewed)
// and touch mapping (crosshair under the finger everywhere on the panel).

static lv_obj_t *test_cross;
static lv_obj_t *test_label;

static void test_screen_touch_cb(lv_event_t *e) {
  lv_indev_t *indev = lv_event_get_indev(e);
  if (!indev) return;
  lv_point_t p;
  lv_indev_get_point(indev, &p);
  lv_obj_set_pos(test_cross, p.x - 20, p.y - 20);
  lv_label_set_text_fmt(test_label, "bibby UI test — touch %d,%d", p.x, p.y);
  fprintf(stderr, "ui-test: touch at %d,%d\n", p.x, p.y);
}

static void test_screen_create(void) {
  lv_obj_t *scr = lv_screen_active();
  lv_obj_set_style_bg_color(scr, lv_color_hex(0x101418), 0);

  lv_obj_t *frame = lv_obj_create(scr);
  lv_obj_set_size(frame, lv_pct(100), lv_pct(100));
  lv_obj_set_style_bg_opa(frame, LV_OPA_0, 0);
  lv_obj_set_style_border_color(frame, lv_color_hex(0x40c060), 0);
  lv_obj_set_style_border_width(frame, 4, 0);
  lv_obj_set_style_radius(frame, 0, 0);

  static const struct { lv_align_t align; const char *txt; } corners[] = {
    { LV_ALIGN_TOP_LEFT, "TL" },    { LV_ALIGN_TOP_RIGHT, "TR" },
    { LV_ALIGN_BOTTOM_LEFT, "BL" }, { LV_ALIGN_BOTTOM_RIGHT, "BR" },
  };
  for (unsigned i = 0; i < 4; i++) {
    lv_obj_t *l = lv_label_create(scr);
    lv_label_set_text(l, corners[i].txt);
    lv_obj_set_style_text_color(l, lv_color_hex(0xe0e4e8), 0);
    lv_obj_align(l, corners[i].align, 0, 0);
  }

  test_label = lv_label_create(scr);
  lv_label_set_text(test_label, "bibby UI test — touch the screen");
  lv_obj_set_style_text_color(test_label, lv_color_hex(0xe0e4e8), 0);
  lv_obj_set_style_text_font(test_label, &lv_font_montserrat_24, 0);
  lv_obj_align(test_label, LV_ALIGN_CENTER, 0, -40);

  test_cross = lv_obj_create(scr);
  lv_obj_set_size(test_cross, 40, 40);
  lv_obj_set_style_radius(test_cross, LV_RADIUS_CIRCLE, 0);
  lv_obj_set_style_bg_color(test_cross, lv_color_hex(0xff5040), 0);
  lv_obj_set_style_border_width(test_cross, 0, 0);

  lv_obj_add_event_cb(scr, test_screen_touch_cb, LV_EVENT_PRESSING, NULL);
}

// ── Entry point ───────────────────────────────────────────────────────────────

int ui_run_mode(BibbyState *st, const BibbyConfig *cfg, bool test_screen) {
  const char *fb = !strcmp(cfg->ui_fb_device, "auto") ? "/dev/fb0"
                                                      : cfg->ui_fb_device;
  // Probe the framebuffer first for a clean fallback path: LVGL's own open
  // failure would leave a half-initialized display behind.
  int probe = open(fb, O_RDWR);
  if (probe < 0) {
    fprintf(stderr, "ui: cannot open %s\n", fb);
    return -1;
  }
  close(probe);

  lv_init();
  lv_tick_set_cb(tick_ms);

  lv_display_t *disp = lv_linux_fbdev_create();
  if (!disp) {
    fprintf(stderr, "ui: fbdev display create failed\n");
    lv_deinit();
    return -1;
  }
  lv_linux_fbdev_set_file(disp, fb);
  lv_display_set_rotation(disp, rotation_from_degrees(cfg->ui_rotation));
  fprintf(stderr, "ui: %s, rotation %d -> UI %dx%d\n", fb, cfg->ui_rotation,
          (int)lv_display_get_horizontal_resolution(disp),
          (int)lv_display_get_vertical_resolution(disp));

  char touch_path[64];
  bool have_touch =
      strcmp(cfg->ui_touch_device, "auto")
          ? (snprintf(touch_path, sizeof(touch_path), "%s", cfg->ui_touch_device), true)
          : find_touch_device(touch_path, sizeof(touch_path));
  if (have_touch) {
    lv_indev_t *touch = lv_evdev_create(LV_INDEV_TYPE_POINTER, touch_path);
    if (touch) fprintf(stderr, "ui: touch %s\n", touch_path);
  } else {
    fprintf(stderr, "ui: no touch device found\n");
  }

  // Second pointer device for the web mirror, fed by the queue the web threads
  // push to. Panel coordinates, exactly like evdev above, so LVGL applies the
  // same rotation to a remote tap as to a finger. Only present when the web
  // server came up with remote control allowed.
  if (cfg->web_allow_control && web_screen_available()) web_screen_attach_indev();

  if (test_screen) test_screen_create();
  else             ui_screen_create(st, cfg);

  while (atomic_load(&st->running)) {
    uint32_t wait_ms = lv_timer_handler();
    if (wait_ms > 30) wait_ms = 30;   // stay responsive to the shutdown flag
    if (wait_ms < 1)  wait_ms = 1;
    usleep(wait_ms * 1000);
  }

  lv_deinit();
  return 0;
}

int ui_run(BibbyState *st, const BibbyConfig *cfg) {
  return ui_run_mode(st, cfg, false);
}

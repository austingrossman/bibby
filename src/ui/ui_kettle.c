#include "ui_kettle.h"

#include <stdlib.h>
#include <string.h>

#include "ui_theme.h"

static lv_obj_t *canvas;
static void     *canvas_buf;
static int32_t   cw, ch;
static float     last_b1 = -1.0f, last_b2 = -1.0f;
static bool      last_grain;

static lv_color_t mix(uint32_t a, uint32_t b, float t) {
  return lv_color_mix(lv_color_hex(b), lv_color_hex(a), (uint8_t)(t * 255.0f));
}

static void fill_rect(lv_layer_t *layer, int32_t x0, int32_t y0, int32_t x1,
                      int32_t y1, uint32_t color, lv_opa_t opa, int32_t radius) {
  lv_draw_rect_dsc_t dsc;
  lv_draw_rect_dsc_init(&dsc);
  dsc.bg_color = lv_color_hex(color);
  dsc.bg_opa   = opa;
  dsc.radius   = radius;
  lv_area_t a = { x0, y0, x1, y1 };
  lv_draw_rect(layer, &dsc, &a);
}

// A heating element: soft halo strokes underneath (growing with brightness),
// then the solid core blended from cold steel grey to hot orange.
static void draw_element(lv_layer_t *layer, int32_t x0, int32_t x1, int32_t y,
                         float bright, int32_t th) {
  lv_draw_line_dsc_t dsc;
  lv_draw_line_dsc_init(&dsc);
  dsc.p1 = (lv_point_precise_t){ x0, y };
  dsc.p2 = (lv_point_precise_t){ x1, y };
  dsc.round_start = 1;
  dsc.round_end   = 1;

  if (bright > 0.01f) {
    dsc.color = lv_color_hex(0xff6e14);
    dsc.opa   = (lv_opa_t)(45.0f * bright);
    dsc.width = th * 3;
    lv_draw_line(layer, &dsc);
    dsc.color = lv_color_hex(0xff821e);
    dsc.opa   = (lv_opa_t)(70.0f * bright);
    dsc.width = th * 2;
    lv_draw_line(layer, &dsc);
  }
  dsc.color = mix(0x524842, 0xff9628, bright);
  dsc.opa   = LV_OPA_COVER;
  dsc.width = th;
  lv_draw_line(layer, &dsc);
}

// Deterministic speckle pattern for the grain bed (same every frame).
static void draw_grain_speckle(lv_layer_t *layer, int32_t x0, int32_t y0,
                               int32_t x1, int32_t y1, int32_t r) {
  uint32_t rng = 0x1234567u;
  for (int i = 0; i < 90; i++) {
    rng = rng * 1664525u + 1013904223u;
    float fx = (float)(rng >> 8) / 16777216.0f;
    rng = rng * 1664525u + 1013904223u;
    float fy = (float)(rng >> 8) / 16777216.0f;
    rng = rng * 1664525u + 1013904223u;
    uint32_t c = (rng >> 8) > 8388608u ? 0x785c32 : 0xb49664;

    int32_t px = x0 + (int32_t)((float)(x1 - x0) * fx);
    int32_t py = y0 + (int32_t)((float)(y1 - y0) * fy);
    fill_rect(layer, px, py, px + r, py + r, c, LV_OPA_COVER, LV_RADIUS_CIRCLE);
  }
}

static void repaint(float b1, float b2, bool grain) {
  lv_canvas_fill_bg(canvas, lv_color_hex(UI_BG), LV_OPA_COVER);

  lv_layer_t layer;
  lv_canvas_init_layer(canvas, &layer);

  const int32_t margin = cw * 8 / 100;
  const int32_t kx0 = margin, kx1 = cw - margin;
  const int32_t kw  = kx1 - kx0;
  const int32_t ky0 = ch * 4 / 100;          // top of walls
  const int32_t ky1 = ch * 97 / 100;         // bottom of kettle
  const int32_t round = kw * 12 / 100;

  // Steel body, then the interior inset by the wall thickness (open top).
  fill_rect(&layer, kx0, ky0, kx1, ky1, 0x3a3e46, LV_OPA_COVER, round);
  const int32_t wt  = kw * 6 / 100;
  const int32_t ix0 = kx0 + wt, ix1 = kx1 - wt;
  const int32_t iy1 = ky1 - wt;
  fill_rect(&layer, ix0, ky0, ix1, iy1, 0x282b31, LV_OPA_COVER, round * 7 / 10);

  // Wort below the fill line; the space above the readouts stays bare steel.
  const int32_t surf = ky0 + (iy1 - ky0) * 38 / 100;
  fill_rect(&layer, ix0, surf, ix1, iy1, 0x60401a, LV_OPA_80, round * 7 / 10);

  // Grain bed: upper part of the contents, only while mashing.
  const int32_t y_div = surf + (iy1 - surf) * 62 / 100;
  if (grain) {
    fill_rect(&layer, ix0, surf, ix1, y_div, 0x967846, LV_OPA_COVER, 4);
    draw_grain_speckle(&layer, ix0, surf, ix1, y_div, kw * 2 / 100);
  }

  // Elements in the free wort under the grain, entering from the right wall.
  const int32_t th = LV_MAX(kw * 5 / 100, 4);
  const int32_t xl = ix0 + kw * 10 / 100;
  const int32_t xr = ix1 - kw * 5 / 100;
  const int32_t ye1 = y_div + (iy1 - y_div) * 40 / 100;
  const int32_t ye2 = y_div + (iy1 - y_div) * 72 / 100;
  draw_element(&layer, xl, xr, ye1, b2, th);   // element 2 above element 1
  draw_element(&layer, xl, xr, ye2, b1, th);

  lv_canvas_finish_layer(canvas, &layer);
}

lv_obj_t *ui_kettle_create(lv_obj_t *parent, int32_t w, int32_t h) {
  cw = w;
  ch = h;
  canvas_buf = malloc((size_t)w * (size_t)h * 4);
  canvas = lv_canvas_create(parent);
  lv_canvas_set_buffer(canvas, canvas_buf, w, h, LV_COLOR_FORMAT_ARGB8888);
  repaint(0.0f, 0.0f, false);
  last_b1 = last_b2 = 0.0f;
  last_grain = false;
  return canvas;
}

void ui_kettle_update(float b1, float b2, bool grain) {
  // Repaint only on a visible change; brightness is quantized so the IIR tail
  // does not repaint forever.
  float q1 = (float)((int)(b1 * 32.0f)) / 32.0f;
  float q2 = (float)((int)(b2 * 32.0f)) / 32.0f;
  if (q1 == last_b1 && q2 == last_b2 && grain == last_grain) return;
  last_b1 = q1;
  last_b2 = q2;
  last_grain = grain;
  repaint(q1, q2, grain);
}

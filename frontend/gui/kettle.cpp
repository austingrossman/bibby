#include "kettle.h"
#include <cmath>
#include <cstdint>

static const float PI = 3.14159265358979f;

// Stroke a polyline as a heating element: a soft halo underneath when energised,
// then the solid core. `on` selects a hot orange over a cold, dim tint.
static void draw_element(ImDrawList *dl, const ImVec2 *pts, int n, bool on, float th) {
  const ImU32 core = on ? IM_COL32(255, 150, 40, 255) : IM_COL32(82, 72, 66, 255);
  if (on) {
    dl->AddPolyline(pts, n, IM_COL32(255, 110, 20, 45), 0, th * 3.0f);
    dl->AddPolyline(pts, n, IM_COL32(255, 130, 30, 70), 0, th * 1.9f);
  }
  dl->AddPolyline(pts, n, core, 0, th);
}

// Deterministic speckle pattern for the grain bed (same every frame, no shimmer).
static void draw_grain_speckle(ImDrawList *dl, float x0, float y0, float x1, float y1, float r) {
  uint32_t rng = 0x1234567u;
  auto next = [&rng]() {
    rng = rng * 1664525u + 1013904223u;
    return (rng >> 8) / 16777216.0f; // [0,1)
  };
  for (int i = 0; i < 70; i++) {
    float px = x0 + (x1 - x0) * next();
    float py = y0 + (y1 - y0) * next();
    ImU32 c  = next() > 0.5f ? IM_COL32(120, 92, 50, 255) : IM_COL32(180, 150, 100, 255);
    dl->AddCircleFilled(ImVec2(px, py), r, c, 6);
  }
}

void draw_kettle(ImDrawList *dl, ImVec2 top_left, ImVec2 size,
                 bool out1, bool out2, bool grain_in) {
  const float margin = size.x * 0.08f;
  const float kx0    = top_left.x + margin;
  const float kx1    = top_left.x + size.x - margin;
  const float kw     = kx1 - kx0;
  const float cx     = (kx0 + kx1) * 0.5f;
  const float ky0    = top_left.y + size.y * 0.10f; // top of walls
  const float ky1    = top_left.y + size.y * 0.92f; // bottom of kettle
  const float round  = kw * 0.20f;

  const ImU32 col_steel   = IM_COL32(58, 62, 70, 255);
  const ImU32 col_outline = IM_COL32(110, 116, 128, 255);
  const ImU32 col_lip     = IM_COL32(92, 98, 110, 255);
  const ImU32 col_liquid  = IM_COL32(96, 64, 26, 210);
  const ImU32 col_grain   = IM_COL32(150, 120, 70, 255);

  // Body (steel walls + back of the cutaway), rounded bottom.
  dl->AddRectFilled(ImVec2(kx0, ky0), ImVec2(kx1, ky1), col_steel, round,
                    ImDrawFlags_RoundCornersBottom);

  // Interior, inset by the wall thickness on the sides and bottom; open at top.
  const float wt     = kw * 0.06f;
  const float ix0    = kx0 + wt;
  const float ix1    = kx1 - wt;
  const float iy0    = ky0;
  const float iy1    = ky1 - wt;
  const float iround = round * 0.7f;

  dl->AddRectFilled(ImVec2(ix0, iy0), ImVec2(ix1, iy1), col_liquid, iround,
                    ImDrawFlags_RoundCornersBottom);

  // Grain bed fills the upper two-thirds of the contents when present.
  const float y_div = iy0 + (iy1 - iy0) * 0.62f;
  if (grain_in) {
    dl->AddRectFilled(ImVec2(ix0, iy0), ImVec2(ix1, y_div), col_grain);
    draw_grain_speckle(dl, ix0, iy0, ix1, y_div, kw * 0.013f);
  }

  // --- Heating elements, in the lower third over the liquid ---
  const float bh = iy1 - y_div;
  const float th = (kw * 0.045f) > 3.0f ? kw * 0.045f : 3.0f;
  const float xL = ix0 + kw * 0.12f;
  const float xR = ix1 - kw * 0.05f; // elements enter through the right wall

  // SSR1 — sling-blade immersion element: two legs joined by a left U-bend.
  {
    const float yc  = y_div + bh * 0.40f;
    const float gap = bh * 0.30f;
    const float r   = gap * 0.5f;
    const float ya  = yc - r;
    const float yb  = yc + r;
    const float xb  = xL + r; // U-bend center x

    ImVec2 pts[20];
    int    n = 0;
    pts[n++] = ImVec2(xR, ya);
    const int arc = 12;
    for (int i = 0; i <= arc; i++) {
      float a = -PI * 0.5f - PI * (float)i / (float)arc; // top -> left -> bottom
      pts[n++] = ImVec2(xb + r * cosf(a), yc + r * sinf(a));
    }
    pts[n++] = ImVec2(xR, yb);
    draw_element(dl, pts, n, out1, th);
  }

  // SSR2 — ripple element: a sine wave across the bottom of the contents.
  {
    const float yc  = y_div + bh * 0.78f;
    const float amp = bh * 0.11f;
    const int   N   = 48;
    ImVec2 pts[N + 1];
    for (int i = 0; i <= N; i++) {
      float fx = (float)i / (float)N;
      pts[i] = ImVec2(xL + (xR - xL) * fx, yc + amp * sinf(2.0f * PI * 3.5f * fx));
    }
    draw_element(dl, pts, N + 1, out2, th);
  }

  // Body outline and a top lip across both walls.
  dl->AddRect(ImVec2(kx0, ky0), ImVec2(kx1, ky1), col_outline, round,
              ImDrawFlags_RoundCornersBottom, 2.0f);
  dl->AddRectFilled(ImVec2(kx0 - margin * 0.3f, ky0 - size.y * 0.022f),
                    ImVec2(kx1 + margin * 0.3f, ky0), col_lip, 3.0f);
}

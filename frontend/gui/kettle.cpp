#include "kettle.h"
#include <cmath>
#include <cstdint>

static const float PI = 3.14159265358979f;

static float clamp01(float v) { return v < 0.0f ? 0.0f : (v > 1.0f ? 1.0f : v); }

// Linear blend between two packed ImU32 colours, component-wise.
static ImU32 lerp_color(ImU32 a, ImU32 b, float t) {
  auto ch = [](ImU32 c, int s) { return (int)((c >> s) & 0xFF); };
  auto mix = [&](int s) { return (int)(ch(a, s) + (ch(b, s) - ch(a, s)) * t); };
  return IM_COL32(mix(IM_COL32_R_SHIFT), mix(IM_COL32_G_SHIFT),
                  mix(IM_COL32_B_SHIFT), mix(IM_COL32_A_SHIFT));
}

// Stroke a polyline as a heating element: a soft halo underneath that grows with
// `bright`, then the solid core blended from a cold tint to a hot orange.
static void draw_element(ImDrawList *dl, const ImVec2 *pts, int n, float bright, float th) {
  bright = clamp01(bright);
  const ImU32 core = lerp_color(IM_COL32(82, 72, 66, 255), IM_COL32(255, 150, 40, 255), bright);
  if (bright > 0.01f) {
    dl->AddPolyline(pts, n, IM_COL32(255, 110, 20, (int)(45.0f * bright)), 0, th * 3.0f);
    dl->AddPolyline(pts, n, IM_COL32(255, 130, 30, (int)(70.0f * bright)), 0, th * 1.9f);
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
                 float bright1, float bright2, bool grain_in, float content_top) {
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
  const ImU32 col_empty   = IM_COL32(40, 43, 49, 255); // bare interior above the liquid
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

  // The contents surface sits at `content_top` (kept below the temperature
  // readout), clamped into the interior. Bare interior above it, liquid below.
  float surf = content_top;
  if (surf < iy0) surf = iy0;
  if (surf > iy1) surf = iy1;

  dl->AddRectFilled(ImVec2(ix0, iy0), ImVec2(ix1, iy1), col_empty, iround,
                    ImDrawFlags_RoundCornersBottom);
  dl->AddRectFilled(ImVec2(ix0, surf), ImVec2(ix1, iy1), col_liquid, iround,
                    ImDrawFlags_RoundCornersBottom);

  // Grain bed fills the upper two-thirds of the contents (below the surface).
  const float y_div = surf + (iy1 - surf) * 0.62f;
  if (grain_in) {
    dl->AddRectFilled(ImVec2(ix0, surf), ImVec2(ix1, y_div), col_grain);
    draw_grain_speckle(dl, ix0, surf, ix1, y_div, kw * 0.013f);
  }

  // --- Heating elements, in the lower third over the liquid ---
  const float bh = iy1 - y_div;
  const float th = (kw * 0.045f) > 3.0f ? kw * 0.045f : 3.0f;
  const float xL = ix0 + kw * 0.12f;
  const float xR = ix1 - kw * 0.05f; // elements enter through the right wall

  // SSR1 — sling-blade immersion element: two legs joined by a left U-bend. Both
  // legs bow the same way (parallel curves) so the blade reads like a scythe.
  {
    const float yc  = y_div + bh * 0.40f;
    const float gap = bh * 0.30f;
    const float r   = gap * 0.5f;
    const float ya  = yc - r;
    const float yb  = yc + r;
    const float xb  = xL + r;        // U-bend center x
    const float bow = bh * 0.08f;    // leg curvature depth (both legs, same sign)

    ImVec2 pts[48];
    int    n = 0;
    const int leg = 12;
    // Top leg: right wall -> U-bend, bowed upward.
    for (int i = 0; i <= leg; i++) {
      float fx = (float)i / (float)leg;
      pts[n++] = ImVec2(xR + (xb - xR) * fx, ya - bow * sinf(PI * fx));
    }
    // U-bend: top -> left -> bottom.
    const int arc = 12;
    for (int i = 1; i < arc; i++) {
      float a = -PI * 0.5f - PI * (float)i / (float)arc;
      pts[n++] = ImVec2(xb + r * cosf(a), yc + r * sinf(a));
    }
    // Bottom leg: U-bend -> right wall, bowed the same way as the top leg. At
    // matching x the two legs share the same bow, so they stay parallel.
    for (int i = 0; i <= leg; i++) {
      float fx = (float)i / (float)leg;
      pts[n++] = ImVec2(xb + (xR - xb) * fx, yb - bow * sinf(PI * fx));
    }
    draw_element(dl, pts, n, bright1, th);
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
    draw_element(dl, pts, N + 1, bright2, th);
  }

  // Body outline and a top lip across both walls.
  dl->AddRect(ImVec2(kx0, ky0), ImVec2(kx1, ky1), col_outline, round,
              ImDrawFlags_RoundCornersBottom, 2.0f);
  dl->AddRectFilled(ImVec2(kx0 - margin * 0.3f, ky0 - size.y * 0.022f),
                    ImVec2(kx1 + margin * 0.3f, ky0), col_lip, 3.0f);
}

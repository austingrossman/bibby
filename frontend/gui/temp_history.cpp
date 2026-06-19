#include "temp_history.h"
#include <algorithm>
#include <cmath>
#include <cstdio>

// Smallest "nice" step >= raw, from a 1 / 2.5 / 5 per-decade sequence
// (..., 0.1, 0.25, 0.5, 1, 2.5, 5, ...). Keeps Y grid lines on readable
// temperatures rather than arbitrary fractions of the panel.
static float nice_step(float raw) {
  if (raw <= 0.0f) return 1.0f;
  float p = powf(10.0f, floorf(log10f(raw)));
  float m = raw / p; // mantissa in [1, 10)
  float nm = (m <= 1.0f) ? 1.0f : (m <= 2.5f) ? 2.5f : (m <= 5.0f) ? 5.0f : 10.0f;
  return nm * p;
}

void TempHistory::push(float temp_c, double now) {
  if (now - last_push_ < SAMPLE_DT_S) return;
  last_push_      = now;
  buf_[head_]     = { now, temp_c };
  head_           = (head_ + 1) % CAP;
  if (count_ < CAP) count_++;
}

void TempHistory::draw_graph(ImDrawList *dl, ImVec2 pos, ImVec2 size, double now) const {
  const ImU32 col_bg     = IM_COL32(20, 22, 26, 255);
  const ImU32 col_border = IM_COL32(90, 95, 105, 255);
  const ImU32 col_grid   = IM_COL32(45, 50, 58, 255);
  const ImU32 col_trace  = IM_COL32(80, 200, 255, 255);
  const ImU32 col_grid_lbl = IM_COL32(150, 156, 168, 255); // grid mark values
  const ImU32 col_bound  = IM_COL32(210, 215, 225, 255);   // axis min/max

  const float lineh = ImGui::GetTextLineHeight();

  // Left gutter holds the temperature axis; the plot sits to its right. Right-
  // aligned label drawn against the plot's left edge at vertical position `yt`.
  const float gutter = ImGui::CalcTextSize("188.88").x + 8.0f;
  const ImVec2 p0 = ImVec2(pos.x + gutter, pos.y);              // plot top-left
  const ImVec2 br = ImVec2(pos.x + size.x, pos.y + size.y);     // plot bottom-right
  const float  pw = br.x - p0.x;
  const float  ph = br.y - p0.y;
  auto label_r = [&](float yt, const char *txt, ImU32 col) {
    dl->AddText(ImVec2(p0.x - 4.0f - ImGui::CalcTextSize(txt).x, yt), col, txt);
  };

  dl->AddRectFilled(p0, br, col_bg, 4.0f);

  // Vertical time grid: 6 columns over the 3-minute window (one line / 30 s).
  const int grid_cols = 6;
  for (int i = 1; i < grid_cols; i++) {
    float gx = p0.x + pw * (float)i / grid_cols;
    dl->AddLine(ImVec2(gx, p0.y), ImVec2(gx, br.y), col_grid, 1.0f);
  }

  // Collect in-window samples (stored oldest -> newest) and find the range.
  const double t_min = now - WINDOW_S;
  ImVec2 pts[CAP];
  int    n   = 0;
  float  lo  = 0.0f, hi = 0.0f;
  int    oldest = (head_ - count_ + CAP) % CAP;
  for (int i = 0; i < count_; i++) {
    const Sample &s = buf_[(oldest + i) % CAP];
    if (s.t < t_min) continue;
    if (n == 0) { lo = hi = s.temp; }
    else        { lo = std::min(lo, s.temp); hi = std::max(hi, s.temp); }
    // Defer pixel mapping until the axis range is known; stash temp in y for now.
    pts[n++] = ImVec2((float)s.t, s.temp);
  }

  // Auto-scale: keep axis limits at least 0.1 °C beyond the data (and give a
  // sane span when the trace is flat).
  float pad      = std::max(0.1f, 0.05f * (hi - lo));
  float axis_min = lo - pad;
  float axis_max = hi + pad;
  float span     = axis_max - axis_min;

  // Horizontal grid on a nice step, aligned to absolute °C values (e.g. 0.25
  // spacing -> lines at 64.75, 65.00, 65.25). The value is drawn on the line
  // itself, at the left end, with the line broken behind the text so it does not
  // strike through. Labels are skipped near the top/bottom where the bounds sit.
  if (n > 0) {
    float step = nice_step(span / 4.0f);
    int   dec  = step < 1.0f ? 2 : (step < 10.0f ? 1 : 0);
    int   k0   = (int)std::ceil(axis_min / step);
    for (int k = k0; k * step <= axis_max + step * 1e-3f; k++) {
      float v  = k * step;
      float gy = p0.y + (1.0f - (v - axis_min) / span) * ph;
      if (gy - p0.y > lineh && br.y - gy > lineh) {
        char b[16];
        snprintf(b, sizeof(b), "%.*f", dec, v);
        float lx = p0.x + 6.0f;
        float tw = ImGui::CalcTextSize(b).x;
        dl->AddText(ImVec2(lx, gy - lineh * 0.5f), col_grid_lbl, b);
        dl->AddLine(ImVec2(lx + tw + 4.0f, gy), ImVec2(br.x, gy), col_grid, 1.0f);
      } else {
        dl->AddLine(ImVec2(p0.x, gy), ImVec2(br.x, gy), col_grid, 1.0f);
      }
    }
  }

  // Map stashed (time, temp) to pixels. X: right edge = now, left = now-WINDOW_S.
  for (int i = 0; i < n; i++) {
    float fx = 1.0f - (float)((now - pts[i].x) / WINDOW_S);
    float fy = 1.0f - (pts[i].y - axis_min) / span;
    pts[i] = ImVec2(p0.x + fx * pw, p0.y + fy * ph);
  }
  if (n >= 2) dl->AddPolyline(pts, n, col_trace, 0, 2.0f);

  dl->AddRect(p0, br, col_border, 4.0f, 0, 1.5f);

  // Axis bounds just outside the plot: max at the top edge, min at the bottom.
  if (n > 0) {
    char top[16], bot[16];
    snprintf(top, sizeof(top), "%.2f", axis_max);
    snprintf(bot, sizeof(bot), "%.2f", axis_min);
    label_r(p0.y + 1.0f, top, col_bound);
    label_r(br.y - lineh - 1.0f, bot, col_bound);
  }
}

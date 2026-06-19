#include "temp_history.h"
#include <algorithm>
#include <cstdio>

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
  const ImU32 col_trace  = IM_COL32(80, 200, 255, 255);
  const ImU32 col_label  = IM_COL32(190, 195, 205, 255);

  ImVec2 br = ImVec2(pos.x + size.x, pos.y + size.y);
  dl->AddRectFilled(pos, br, col_bg, 4.0f);

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

  // Map stashed (time, temp) to pixels. X: right edge = now, left = now-WINDOW_S.
  for (int i = 0; i < n; i++) {
    float fx = 1.0f - (float)((now - pts[i].x) / WINDOW_S);
    float fy = 1.0f - (pts[i].y - axis_min) / span;
    pts[i] = ImVec2(pos.x + fx * size.x, pos.y + fy * size.y);
  }
  if (n >= 2) dl->AddPolyline(pts, n, col_trace, 0, 2.0f);

  dl->AddRect(pos, br, col_border, 4.0f, 0, 1.5f);

  // Axis labels: max at top-left, min at bottom-left (only when we have data).
  if (n > 0) {
    char top[16], bot[16];
    snprintf(top, sizeof(top), "%.1f", axis_max);
    snprintf(bot, sizeof(bot), "%.1f", axis_min);
    dl->AddText(ImVec2(pos.x + 4.0f, pos.y + 2.0f), col_label, top);
    float lh = ImGui::GetTextLineHeight();
    dl->AddText(ImVec2(pos.x + 4.0f, br.y - lh - 2.0f), col_label, bot);
  }
}

#pragma once
#include <imgui.h>

// Rolling temperature history for the on-screen graph. Holds the last
// TempHistory::WINDOW_S seconds of samples in a fixed circular buffer, throttled
// so a 60 fps caller does not flood it. draw_graph() auto-scales the Y axis so
// the plotted limits sit at least 0.1 °C beyond the data min/max.
class TempHistory {
public:
  static constexpr double WINDOW_S      = 180.0; // 3 minutes
  static constexpr double SAMPLE_DT_S   = 0.5;   // one stored point per 0.5 s
  static constexpr int    CAP           = 400;   // > WINDOW_S / SAMPLE_DT_S

  // Record a sample taken at monotonic time `now` (seconds). Samples arriving
  // sooner than SAMPLE_DT_S after the previous stored one are dropped.
  void push(float temp_c, double now);

  // Draw the trace and auto-scaled axis into the rectangle [pos, pos+size].
  void draw_graph(ImDrawList *dl, ImVec2 pos, ImVec2 size, double now) const;

private:
  struct Sample { double t; float temp; };
  Sample buf_[CAP];
  int    head_  = 0;   // index of next write
  int    count_ = 0;   // valid samples, <= CAP
  double last_push_ = -1.0e9;
};

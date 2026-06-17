#include "temp_filter.h"
#include <algorithm>

SecondOrderAverage::SecondOrderAverage(int window)
  : window_(window), buf1_(window, 0.0f), buf2_(window, 0.0f) {}

// Advance one boxcar stage: drop the oldest sample, add the new one, return mean.
float SecondOrderAverage::run_stage(std::vector<float> &buf, int &idx, float &sum, float in) {
  sum -= buf[idx];
  buf[idx] = in;
  sum += in;
  idx = (idx + 1) % window_;
  return sum / window_;
}

float SecondOrderAverage::push(float sample) {
  if (!primed_) {
    std::fill(buf1_.begin(), buf1_.end(), sample);
    std::fill(buf2_.begin(), buf2_.end(), sample);
    sum1_ = sum2_ = sample * window_;
    primed_ = true;
    return value_ = sample;
  }
  float stage1 = run_stage(buf1_, idx1_, sum1_, sample);
  return value_ = run_stage(buf2_, idx2_, sum2_, stage1);
}

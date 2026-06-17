#pragma once
#include <vector>

// Second-order moving-average filter: two cascaded boxcar stages, equivalent to
// a second-order CIC running at one sample per input (triangular weighting).
// The first sample primes both stages so the output starts at the real value
// instead of ramping up from zero.
class SecondOrderAverage {
public:
  explicit SecondOrderAverage(int window);

  float push(float sample); // feed a sample, returns the new filtered value
  float value() const { return value_; }

private:
  float run_stage(std::vector<float> &buf, int &idx, float &sum, float in);

  int                window_;
  std::vector<float> buf1_, buf2_;
  int                idx1_ = 0, idx2_ = 0;
  float              sum1_ = 0.0f, sum2_ = 0.0f;
  bool               primed_ = false;
  float              value_  = 0.0f;
};

#pragma once
#include <string>

class Max31865 {
public:
  explicit Max31865(const std::string &spidev_path, int ref_resistor = 400);
  float read_temperature(); // returns °C, returns NaN on fault

private:
  int   fd_;
  int   ref_resistor_;
  float rtd_nominal_ = 100.0f; // PT100
};

#pragma once

#include <chrono>
#include <iostream>

class Timer
{
protected:
  const std::string mLabel;
  const std::chrono::high_resolution_clock::time_point mStart;
public:
  Timer(const std::string &label);

  ~Timer(void);
};
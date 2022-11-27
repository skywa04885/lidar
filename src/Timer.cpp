#include "Timer.hpp"

Timer::Timer(
    const std::string &label) : mLabel(label),
                                mStart(std::chrono::high_resolution_clock::now())
{
}

Timer::~Timer(void)
{
  const std::chrono::time_point end = std::chrono::high_resolution_clock::now();
  const std::chrono::nanoseconds nsDuration = end - mStart;
  const std::chrono::microseconds microDuration = std::chrono::duration_cast<std::chrono::microseconds>(nsDuration);
  const std::chrono::milliseconds msDuration = std::chrono::duration_cast<std::chrono::milliseconds>(nsDuration);
  std::cout << "TIMER [" << mLabel << "]: "
            << "Elapsed: " << nsDuration.count() << " nanoseconds, " << microDuration.count() << " microseconds, " << msDuration.count() << " milliseconds" << std::endl;
}
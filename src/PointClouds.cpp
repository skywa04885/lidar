#include "PointClouds.hpp"

PointClouds::MapperOptions::MapperOptions(
    double learningRate,
    double exitThreshold) noexcept : learningRate(learningRate),
                                     exitThreshold(exitThreshold)
{
}

PointClouds::MapperOptions PointClouds::MapperOptions::defaults(void) noexcept
{
  return PointClouds::MapperOptions(0.001, 0.000001);
}

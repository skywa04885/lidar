#pragma once

#include <cstddef>

#include "RDTree.hpp"

namespace PointClouds
{
  /// @brief a structure containing the options for a mapper.
  struct MapperOptions
  {
  public:
    double learningRate, exitThreshold;

  public:
    /// @brief constructs a new mapper options structure.
    /// @param learningRate the learning rate.
    /// @param exitThreshold the threshold to exit.
    MapperOptions(
        double learningRate,
        double exitThreshold) noexcept;

  public:
    /// @brief constructs a mapper options struct with default values.
    /// @return the constructed mapper options.
    static MapperOptions defaults(void) noexcept;
  };

  class Mapper2D
  {
  private:
    MapperOptions mOptions;

  public:
    /// @brief constructs a new mapper.
    /// @param options the options for the mapper.
    Mapper2D(const MapperOptions &options) : mOptions(options)
    {
    }

  public:

    double orientational(RDTree<double, 2> &target, RDTree<double, 2> &reference) const {
      if (target.isEmpty() || reference.isEmpty())
      {
        throw std::runtime_error("The target and reference tree must not be empty.");
      }

      std::vector<std::tuple<std::array<double, 2>, std::array<double, 2>>> temp;
      temp.reserve(target.getSize());

      std::for_each(target.begin(), target.end(), [&](const std::array<double, 2> &targetPoint) {

      });
    }
  };
}
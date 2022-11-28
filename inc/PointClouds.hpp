#pragma once

#include <cstddef>
#include <tuple>
#include <vector>

#include "KDTree.hpp"

namespace PointClouds
{
  template <typename T>
  class PointCloud2D
  {
  protected:
    KDTree<T, 2> mTree;
  };

  template <typename T>
  struct MapperOptions
  {
  public:
    double learningRate, exitThreshold;

  public:
    /// @brief constructs a new mapper options structure.
    /// @param learningRate the learning rate.
    /// @param exitThreshold the threshold to exit.
    MapperOptions(double learningRate, double exitThreshold) noexcept;

  public:
    /// @brief constructs a mapper options struct with default values.
    /// @return the constructed mapper options.
    static MapperOptions defaults(void) noexcept;
  };

  template <typename T>
  class Mapper2D
  {
  private:
    MapperOptions<T> mOptions;

  public:
    /// @brief constructs a new mapper.
    /// @param options the options for the mapper.
    Mapper2D(const MapperOptions<T> &options) : mOptions(options) {}

  public:
    std::vector<std::tuple<std::array<T, 2>, std::array<T, 2>>>
    getNearestPointPairs(const KDTree<T, 2> &targetTree,
                         const KDTree<T, 2> &referenceTree)
    {
      std::vector<std::tuple<std::array<T, 2>, std::array<T, 2>>> result;
      result.reserve(targetTree.getSize());

      std::for_each(targetTree.begin(), targetTree.end(),
                    [&](const std::array<T, 2> &targetPoint)
                    {
                      const auto &nearestReferencePoint =
                          referenceTree.nearest(targetPoint);

                      result.push_back(nearestReferencePoint);
                    });
      return result;
    }

    T orientational(KDTree<T, 2> &target, KDTree<T, 2> &reference) const
    {
      if (target.isEmpty() || reference.isEmpty())
      {
        throw std::runtime_error(
            "The target and reference tree must not be empty.");
      }

      auto nearestPointPairs = getNearestPointPairs(target, reference);
    }
  };
} // namespace PointClouds

#pragma once

#include <memory>
#include <stdexcept>
#include <iostream>
#include <string>
#include <tuple>
#include <optional>
#include <cmath>

#include "sl_lidar_cmd.h"
#include "sl_lidar_driver.h"
#include "sl_types.h"

#define HANDLE_LIDAR_ERROR(EXPR) __handleLidarError(#EXPR, EXPR)

inline void __handleLidarError(const char *const expr, const sl_result result)
{
  if (SL_IS_OK(result))
  {
    return;
  }

  std::string error("(");
  error.append(expr);
  error.append(") failed: ");

  switch (result)
  {
  case SL_RESULT_OPERATION_TIMEOUT:
    error.append("Timeout occured.");
    break;
  case SL_RESULT_OPERATION_FAIL:
    error.append("Failed.");
    break;
  default:
    error.append(std::to_string(result));
    break;
  }

  throw std::runtime_error(error);
}

class Lidar
{
protected:
  sl::ILidarDriver *mLidarDriver;
  std::vector<sl::LidarScanMode> mModes;
  size_t mMode;

public:
  Lidar(sl::ILidarDriver *lidarDriver, const std::vector<sl::LidarScanMode> &modes) : mLidarDriver(lidarDriver), mModes(modes), mMode(0) {}

  ~Lidar(void)
  {
    // Stops the lidar.
    HANDLE_LIDAR_ERROR(mLidarDriver->stop());

    // Disconnects the lidar.
    mLidarDriver->disconnect();

    // Deletes the driver.
    delete mLidarDriver;
  }

public:
  inline Lidar &setMode(const size_t mode) noexcept
  {
    mMode = mode;

    return *this;
  }

  inline Lidar &setMode(const std::string &name)
  {
    const std::vector<sl::LidarScanMode>::iterator it = std::find_if(mModes.begin(), mModes.end(), [&name](const sl::LidarScanMode &mode)
                                                                     { return mode.scan_mode == name; });

    if (it == mModes.end())
      throw std::runtime_error("Could not get mode by given name.");

    mMode = it->id;

    return *this;
  }

public:
  static Lidar serial(const std::string &device, const int baudrate)
  {
    sl::IChannel *channel;
    sl::ILidarDriver *driver;

    channel = *sl::createSerialPortChannel(device, baudrate);
    driver = *sl::createLidarDriver();

    // Connects the driver to the lidar.
    HANDLE_LIDAR_ERROR(driver->connect(channel));

    // Sets the default motor speed.
    HANDLE_LIDAR_ERROR(driver->setMotorSpeed());

    // Gets the modes.
    std::vector<sl::LidarScanMode> modes;
    HANDLE_LIDAR_ERROR(driver->getAllSupportedScanModes(modes));

    return Lidar(driver, modes);
  }

public:
  sl_lidar_response_device_info_t getInfo()
  {
    sl_lidar_response_device_info_t info;

    HANDLE_LIDAR_ERROR(mLidarDriver->getDeviceInfo(info));

    return info;
  }

  sl_lidar_response_device_health_t getHealth()
  {
    sl_lidar_response_device_health_t health;

    HANDLE_LIDAR_ERROR(mLidarDriver->getHealth(health));

    return health;
  }

  std::tuple<size_t, std::shared_ptr<sl_lidar_response_measurement_node_hq_t[]>>
  scan()
  {
    // Starts the scan.
    HANDLE_LIDAR_ERROR(mLidarDriver->startScanExpress(false, mModes[mMode].id));

    // Since the scan was started successfully, allocate the required memory.
    auto result = std::shared_ptr<sl_lidar_response_measurement_node_hq_t[]>(
        new sl_lidar_response_measurement_node_hq_t[8192]);
    size_t resultLength = 8192;

    // Grabs the scan data.
    HANDLE_LIDAR_ERROR(
        mLidarDriver->grabScanDataHq(result.get(), resultLength));

    // Orders the scan data.
    HANDLE_LIDAR_ERROR(
        mLidarDriver->ascendScanData(result.get(), resultLength));

    // Returns the result.
    return std::make_tuple(resultLength, result);
  }
};

template <typename T>
class LidarCoordinateConverter;

template <typename T>
class LidarCoordinateConverterBuilder
{
protected:
  std::optional<T> mTranslateX, mTranslateY;
  std::optional<T> mRotationAngle;
  bool mDisposeZeroDistances;

public:
  LidarCoordinateConverterBuilder() : mTranslateX(std::nullopt),
                                      mTranslateY(std::nullopt),
                                      mRotationAngle(std::nullopt),
                                      mDisposeZeroDistances(true)
  {
  }

public:
  LidarCoordinateConverterBuilder<T> &translateX(const T translateX)
  {
    mTranslateX = translateX;
    return *this;
  }

  LidarCoordinateConverterBuilder<T> &translateY(const T translateY)
  {
    mTranslateY = translateY;
    return *this;
  }

  LidarCoordinateConverterBuilder<T> &rotate(const T angle)
  {
    mRotationAngle = angle;
    return *this;
  }

  LidarCoordinateConverterBuilder<T> &rotate(const bool disposeZeroDistances)
  {
    mDisposeZeroDistances = disposeZeroDistances;
    return *this;
  }

public:
  LidarCoordinateConverter<T> build(
      std::shared_ptr<sl_lidar_response_measurement_node_hq_t[]> nodes,
      size_t nodeCount) const
  {
    return LidarCoordinateConverter<T>(mTranslateX, mTranslateY, mRotationAngle,
                                nodes, nodeCount, mDisposeZeroDistances);
  }
};

template <typename T>
class LidarCoordinateConverter
{
protected:
  std::optional<T> mTranslateX, mTranslateY;
  std::optional<T> mRotationAngle;
  std::shared_ptr<sl_lidar_response_measurement_node_hq_t[]> mNodes;
  size_t mNodeCount;
  bool mDisposeZeroDistances;

public:
  LidarCoordinateConverter(
      std::optional<T> translateX,
      std::optional<T> translateY,
      std::optional<T> rotationAngle,
      std::shared_ptr<sl_lidar_response_measurement_node_hq_t[]> nodes,
      size_t nodeCount,
      bool disposeZeroDistances) : mTranslateX(translateX), mTranslateY(translateY),
                                   mRotationAngle(rotationAngle), mNodes(nodes), mNodeCount(nodeCount),
                                   mDisposeZeroDistances(disposeZeroDistances) {}

public:
  class iterator
  {
    using iterator_category = std::input_iterator_tag;
    using value_type = std::array<T, 2>;

  protected:
    LidarCoordinateConverter<T> *mConverter;
    size_t mIndex;

  public:
    iterator(LidarCoordinateConverter<T> *converter, size_t index) : mConverter(converter), mIndex(index)
    {
    }

  public:
    iterator &operator++(void)
    {
      while (mIndex < mConverter->mNodeCount and mConverter->mDisposeZeroDistances and mConverter->mNodes[++mIndex].dist_mm_q2 == 0)
      {
      };

      return *this;
    }

    iterator operator++(int)
    {
      iterator result = *this;

      ++(*this);

      return result;
    }

    bool operator==(const iterator &other) const noexcept
    {
      return (mConverter == other.mConverter) && (mIndex == other.mIndex);
    }

    bool operator!=(const iterator &other) const noexcept
    {
      return !((*this) == other);
    }

    value_type operator*(void)
    {
      const sl_lidar_response_measurement_node_hq_t *node = &mConverter->mNodes[mIndex];

      T angle =
          (((static_cast<T>(node->angle_z_q14) * static_cast<T>(90)) / static_cast<T>(16384)) /
           static_cast<T>(180)) *
          std::numbers::pi_v<T>;

      if (this->mConverter->mRotationAngle.has_value())
        angle += *this->mConverter->mRotationAngle;

      const T distance = (static_cast<T>(node->dist_mm_q2) / static_cast<T>(4));

      std::array<T, 2> result = {
          distance * std::cos(angle),
          distance * std::sin(angle),
      };

      if (this->mConverter->mTranslateX.has_value())
        result.at(0) += *this->mConverter->mTranslateX;

      if (this->mConverter->mTranslateY.has_value())
        result.at(1) += *this->mConverter->mTranslateY;

      return result;
    }
  };

public:
  iterator begin(void) noexcept
  {
    return iterator(this, 0);
  }

  iterator end(void) noexcept
  {
    return iterator(this, mNodeCount);
  }

public:
  static inline LidarCoordinateConverterBuilder<T> builder(void) noexcept
  {
    return LidarCoordinateConverterBuilder<T>();
  }
};

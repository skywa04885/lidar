#pragma once

#include <memory>
#include <stdexcept>
#include <iostream>
#include <string>
#include <tuple>

#include "sl_lidar_cmd.h"
#include "sl_lidar_driver.h"
#include "sl_types.h"

#define HANDLE_LIDAR_ERROR(EXPR) __handleLidarError(#EXPR, EXPR)

inline void __handleLidarError(const char *const expr, const sl_result result) {
  if (SL_IS_OK(result)) {
    return;
  }

  std::string error("(");
  error.append(expr);
  error.append(") failed: ");

  switch (result) {
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

class Lidar {
protected:
  sl::ILidarDriver *mLidarDriver;

public:
  Lidar(sl::ILidarDriver *lidarDriver) : mLidarDriver(lidarDriver) {}

  ~Lidar(void) {
    // Stops the lidar.
    HANDLE_LIDAR_ERROR(mLidarDriver->stop());

    // Disconnects the lidar.
    mLidarDriver->disconnect();

    // Deletes the driver.
    delete mLidarDriver;
  }
public:
  static Lidar serial(const std::string &device, const int baudrate) {
    sl::IChannel *channel;
    sl::ILidarDriver *driver;

    channel = *sl::createSerialPortChannel(device, baudrate);
    driver = *sl::createLidarDriver();

    // Connects the driver to the lidar.
    HANDLE_LIDAR_ERROR(driver->connect(channel));

    // Sets the default motor speed.
    HANDLE_LIDAR_ERROR(driver->setMotorSpeed());

    return Lidar(driver);
  }

public:
  sl_lidar_response_device_info_t getInfo() {
    sl_lidar_response_device_info_t info;

    HANDLE_LIDAR_ERROR(mLidarDriver->getDeviceInfo(info));

    return info;
  }

  sl_lidar_response_device_health_t getHealth() {
    sl_lidar_response_device_health_t health;

    HANDLE_LIDAR_ERROR(mLidarDriver->getHealth(health));

    return health;
  }

  std::tuple<size_t, std::unique_ptr<sl_lidar_response_measurement_node_hq_t[]>>
  scan() {
    // Starts the scan.
    HANDLE_LIDAR_ERROR(mLidarDriver->startScan(false, true));

    // Since the scan was started successfully, allocate the required memory.
    auto result = std::unique_ptr<sl_lidar_response_measurement_node_hq_t[]>(
        new sl_lidar_response_measurement_node_hq_t[8192]
        );
    size_t resultLength = 8192;

    // Grabs the scan data.
    HANDLE_LIDAR_ERROR(
        mLidarDriver->grabScanDataHq(result.get(), resultLength));

    // Orders the scan data.
    HANDLE_LIDAR_ERROR(
        mLidarDriver->ascendScanData(result.get(), resultLength));

    // Returns the result.
    return std::make_tuple(resultLength, std::move(result));
  }
};

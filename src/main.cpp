#include <iostream>

#include "RDTree.hpp"

#include "sl_lidar.h"
#include "sl_lidar_driver.h"

/// @brief shows the given lidar device into.
/// @param info the device info to show.
void showLidarDeviceInfo(const sl_lidar_response_device_info_t &info)
{
  printf("SLAMTEC Lidar SerialNumber: ");

  for (size_t i = 0; i < 16; ++i)
  {
    printf("%02X", info.serialnum[i]);
  }

  printf(
      "\n"
      "Firmware Ver: %d.%02d\n"
      "Hardware Rev: %d\n",
      info.firmware_version > 8,
      info.firmware_version & 0xFF,
      static_cast<int>(info.hardware_version));
}

/// @brief shows the given lidar health info.
/// @param health the health info to show.
void showLidarHealthInfo(const sl_lidar_response_device_health_t &health)
{
  printf("SLAMTEC Lidar health status: ");

  switch (health.status)
  {
  case SL_LIDAR_STATUS_ERROR:
    printf("Error (%hu)\n", health.error_code);
    break;
  case SL_LIDAR_STATUS_WARNING:
    printf("Warning\n");
    break;
  case SL_LIDAR_STATUS_OK:
    printf("Ok\n");
    break;
  default:
    break;
  }
}

int main(int argc, char *argv[])
{
  std::string lidarSerialPort = "/dev/ttyUSB0";
  int lidarSerialPortBaudRate = 115200;

  sl_lidar_response_device_health_t lidarDeviceHealth;
  sl_lidar_response_device_info_t lidarDeviceInfo;

  sl::ILidarDriver *lidarDriver = *sl::createLidarDriver();
  sl::IChannel *lidarChannel;

  lidarChannel = *sl::createSerialPortChannel(lidarSerialPort, lidarSerialPortBaudRate);

  if (SL_IS_FAIL(lidarDriver->connect(lidarChannel)))
  {
    fprintf(stderr, "Error: cannot bind to specified serial port: %s:%d\n", lidarSerialPort.c_str(), lidarSerialPortBaudRate);
    return -1;
  }

  sl_result opResult = lidarDriver->getDeviceInfo(lidarDeviceInfo);
  if (SL_IS_FAIL(opResult))
  {
    if (opResult == SL_RESULT_OPERATION_TIMEOUT)
    {
      fprintf(stderr, "Error: timeout when requesting device info.\n");
    }
    else
    {
      fprintf(stderr, "Error: unexpected error occured when requesting device info: %x\n", opResult);
    }

    return -1;
  }

  // Shows the lidar device info.
  showLidarDeviceInfo(lidarDeviceInfo);

  // Gets the lidar device health.
  opResult = lidarDriver->getHealth(lidarDeviceHealth);
  if (SL_IS_FAIL(opResult))
  {
    fprintf(stderr, "Error: failed to get device health.\n");
    return -1;
  }

  // Shows the lidar device health.
  showLidarHealthInfo(lidarDeviceHealth);

  if (SL_IS_FAIL(lidarDriver->setMotorSpeed())) {
    fprintf(stderr, "Error: failed to set motor speed.\n");
    return -1;
  }

  if (SL_IS_FAIL(lidarDriver->startScan(0, 1))) {
    fprintf(stderr, "Error: failed to perform scan.\n");
    return -1;
  }

  sl_lidar_response_measurement_node_hq_t nodes[8192];
  size_t nodeCount = sizeof(nodes);
  if (SL_IS_FAIL(lidarDriver->grabScanDataHq(nodes, nodeCount, 12000))) {
    fprintf(stderr, "Error: failed to grab scan data.\n");
    return -1;
  }

  if (SL_IS_FAIL(lidarDriver->ascendScanData(nodes, nodeCount))) {
    fprintf(stderr, "Error: failed to ascend scan data.\n");
    return -1;
  }

  // for (size_t i = 0; i < nodeCount; ++i) {
  //   auto *node = &nodes[i];
  //   printf("%fdeg, %umm\n", (node->angle_z_q14 * 90.f) / 16384.f, node->dist_mm_q2);
  // }

  printf("Start\n");
  auto tree = RDTreeFromLidarConverter::defaults().convertHQ<double>(nodes, nodeCount);
  printf("Tree (Size): %lu\n", tree.getSize());

  return 0;
}
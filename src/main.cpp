#include <chrono>
#include <cstring>
#include <iostream>
#include <memory>
#include <tuple>

#include "Timer.hpp"
#include "Lidar.hpp"
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

  printf("\n"
         "Firmware Ver: %d.%02d\n"
         "Hardware Rev: %d\n",
         info.firmware_version > 8, info.firmware_version & 0xFF,
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

template <typename T, size_t N>
std::ostream &operator<<(std::ostream &stream, const std::array<T, N> &array)
{
  auto it = array.begin();
  auto end = array.end();

  stream << "{";
  while (it != end)
  {
    stream << *it;

    if (++it != end)
    {
      stream << ", ";
    }
  }

  stream << "}";

  return stream;
}

template <typename T>
void printRdtree(RDTree<T, 2> &tree, size_t width, size_t height)
{
  char point = '*';
  char empty = ' ';

  auto [min, max] = tree.getRange();
  std::cout << "Min: " << min << ", Max: " << max << std::endl;

  std::unique_ptr<char[]> screen =
      std::unique_ptr<char[]>(new char[width * height]);
  std::memset(screen.get(), (int)empty, width * height);

  for (const auto node : tree)
  {
    auto &data = node->getData();

    T rowPercentage = (data.at(1) - min.at(1)) / (max.at(1) - min.at(1));
    size_t row =
        static_cast<size_t>(static_cast<T>(height - 1) * rowPercentage);

    T colPercentage = (data.at(0) - min.at(0)) / (max.at(0) - min.at(0));
    size_t col = static_cast<size_t>(static_cast<T>(width - 1) * colPercentage);

    screen.get()[row * width + col] = point;
  }

  for (size_t row = 0; row < height; ++row)
  {
    for (size_t col = 0; col < width; ++col)
    {
      std::cout << screen.get()[row * width + col];
    }

    std::cout << std::endl;
  }
}

int main(int argc, char *argv[])
{
  std::string lidarSerialPort = "/dev/ttyUSB0";
  int lidarSerialPortBaudRate = 115200;

  Lidar lidar = Lidar::serial(lidarSerialPort, lidarSerialPortBaudRate);

  const auto lidarInfo = lidar.getInfo();
  showLidarDeviceInfo(lidarInfo);

  const auto lidarHealth = lidar.getHealth();
  showLidarHealthInfo(lidarHealth);

  for (;;)
  {
    auto [nodeCount, nodes] = lidar.scan();

    auto tree = RDTreeFromLidarConverter::defaults().convertHQ<double>(
        nodes.get(), nodeCount);
    printRdtree(tree, 60, 30);
  }

  return 0;
}

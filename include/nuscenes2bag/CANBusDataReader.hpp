#pragma once

#include <iostream>
#include <map>
#include <set>

#include <nlohmann/json.hpp>

#include "nuscenes2bag/CANBusDataTypes.hpp"
#include "nuscenes2bag/Filesystem.hpp"
#include "nuscenes2bag/ToDebugString.hpp"

#include <boost/optional.hpp>

namespace nuscenes2bag {
class CANBusDataReader {
public:
  void loadFromDirectory(const std::vector<uint32_t> &sceneNumberVec, const fs::path &directoryPath);
  std::vector<IMUData> loadIMUDataFromFile(const fs::path& filePath);
  std::vector<WheelSpeedData> loadWheelSpeedDataFromFile(const fs::path& filePath);

  std::vector<IMUData> getIMUData(const uint32_t &sceneNumber) const;
  std::vector<WheelSpeedData> getWheelSpeedData(const uint32_t &sceneNumber) const;

private:
  nlohmann::json slurpJsonFile(const fs::path& filePath);

  std::map<uint32_t, std::vector<IMUData>> imu_data_;
  std::map<uint32_t, std::vector<WheelSpeedData>>  wheel_speed_data_;
};
}
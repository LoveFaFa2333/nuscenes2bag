#include "nuscenes2bag/utils.hpp"
#include <nuscenes2bag/CANBusDataReader.hpp>

#include <algorithm>
#include <fstream>
#include <iostream>
#include <map>
#include <regex>

using namespace std;
namespace json = nlohmann;

namespace nuscenes2bag {

nlohmann::json CANBusDataReader::slurpJsonFile(const fs::path& filePath)
{
  std::ifstream file(filePath.string());
  if (!file.is_open()) {
    std::string errMsg = string("Unable to open ") + filePath.string();
    throw std::runtime_error(errMsg);
  }
  json::json newJson;
  file >> newJson;
  return newJson;
}

std::vector<IMUData> CANBusDataReader::loadIMUDataFromFile(const fs::path& filePath) {
    auto IMUDataJsons = slurpJsonFile(filePath);
    std::vector<IMUData> imu_data;

    for (const auto& IMUJson : IMUDataJsons) {
        IMUData ms_imu;
        ms_imu.linear_accel[0] = IMUJson["linear_accel"][0];
        ms_imu.linear_accel[1] = IMUJson["linear_accel"][1];
        ms_imu.linear_accel[2] = IMUJson["linear_accel"][2];

        ms_imu.rotation_rate[0] = IMUJson["rotation_rate"][0];
        ms_imu.rotation_rate[1] = IMUJson["rotation_rate"][1];
        ms_imu.rotation_rate[2] = IMUJson["rotation_rate"][2];

        ms_imu.q[0] = IMUJson["q"][0];
        ms_imu.q[1] = IMUJson["q"][1];
        ms_imu.q[2] = IMUJson["q"][2];
        ms_imu.q[3] = IMUJson["q"][3];

        ms_imu.utime = IMUJson["utime"];
        
        imu_data.push_back(ms_imu);
    }

    return imu_data;
}

std::vector<WheelSpeedData> CANBusDataReader::loadWheelSpeedDataFromFile(const fs::path& filePath) {
    auto VehInfoJsons = slurpJsonFile(filePath);
    std::vector<WheelSpeedData> wheel_speed_data;

    for (const auto& VehInfoJson : VehInfoJsons) {
        WheelSpeedData wheel_speed;
        wheel_speed.utime = VehInfoJson["utime"];
        wheel_speed.left_speed = VehInfoJson["RL_wheel_speed"];
        wheel_speed.right_speed = VehInfoJson["RR_wheel_speed"];
        wheel_speed.CalVelAndYawRate();

        wheel_speed_data.push_back(wheel_speed);
    }

    return wheel_speed_data;
}

std::vector<IMUData> CANBusDataReader::getIMUData(const uint32_t &sceneNumber) const {
    auto const iter = imu_data_.find(sceneNumber);
    if(iter == imu_data_.end()) {
        std::cout << "Cannot find imu data from scene-" << sceneNumber << std::endl;
    }
    return  iter->second;
}

std::vector<WheelSpeedData> CANBusDataReader::getWheelSpeedData(const uint32_t &sceneNumber) const {
    auto const iter = wheel_speed_data_.find(sceneNumber);
    if(iter == wheel_speed_data_.end()) {
        std::cout << "Cannot find wheel speed data from scene-" << sceneNumber << std::endl;
    }
    return  iter->second;
}

void CANBusDataReader::loadFromDirectory(const std::vector<uint32_t> &sceneNumberVec, const fs::path& directoryPath)
{   
    for(auto const sceneNumber : sceneNumberVec) {
        std::ostringstream number_str;
        number_str << std::setw(4) << std::setfill('0') << sceneNumber; 
        const std::string prefix = "scene-" + number_str.str(); 
        const fs::path metaFile = directoryPath / (prefix + "_meta.json");
        const fs::path imuFile = directoryPath / (prefix + "_ms_imu.json");
        const fs::path poseFile = directoryPath / (prefix + "_pose.json");
        const fs::path routeFile = directoryPath / (prefix + "_route.json");
        const fs::path steerAngleFile = directoryPath / (prefix + "_steeranglefeedback.json");
        const fs::path vehicleMonitorFile = directoryPath / (prefix + "_vehicle_monitor.json");
        const fs::path zoeSensorsFile = directoryPath / (prefix + "_zoesensors.json");
        const fs::path zoeVehInfoFile = directoryPath / (prefix + "_zoe_veh_info.json");

        // only use imu and wheel speed data
        imu_data_.emplace(sceneNumber, loadIMUDataFromFile(imuFile));
        wheel_speed_data_.emplace(sceneNumber, loadWheelSpeedDataFromFile(zoeVehInfoFile));
    }
}
}
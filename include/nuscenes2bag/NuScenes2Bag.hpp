#pragma once

#include <rosbag/bag.h>

#include "nuscenes2bag/CANBusDataReader.hpp"
#include "nuscenes2bag/DatasetTypes.hpp"
#include "nuscenes2bag/Filesystem.hpp"

#include <boost/optional.hpp>


namespace nuscenes2bag {

enum SensorType {
  LIDAR_TOP = 1,
  CAM_FRONT = 2,
  CAM_FRONT_LEFT = 3,
  CAM_FRONT_RIGHT = 4,
  CAM_BACK = 5,
  CAM_BACK_LEFT = 6,
  CAM_BACK_RIGHT = 7,
  RADAR_FRONT = 8, 
  RADAR_FRONT_LEFT = 9, 
  RADAR_FRONT_RIGHT = 10, 
  RADAR_BACK_LEFT = 11, 
  RADAR_BACK_RIGHT = 12, 
  MS_IMU = 13,
  WHEEL_SPEED = 14
};

struct NuScenes2Bag {

public:
  NuScenes2Bag();

  void convertDirectory(const fs::path &inDatasetPath,
                        const std::string& version,
                        const fs::path &outputRosbagPath,
                        int32_t threadNumber,
                        std::vector<uint32_t> sceneNumberVec,
                        const bool &enable_lidar = true,
                        const bool &enable_cam = false,
                        const bool &enable_radar = false
                        );

private:
  std::string inDatasetPathString;
};

}
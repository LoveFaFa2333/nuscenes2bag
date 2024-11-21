#pragma once

#include "nuscenes2bag/Filesystem.hpp"
#include "nuscenes2bag/MetaDataReader.hpp"
#include "nuscenes2bag/FileProgress.hpp"
#include "nuscenes2bag/CANBusDataReader.hpp"

#include "rosbag/bag.h"

namespace nuscenes2bag {

class SceneConverter {
    public:
    SceneConverter(const MetaDataProvider& metaDataProvider, const CANBusDataReader& canBusDataReader);

    void submit(const Token& sceneToken, FileProgress& fileProgress);

    void run(const fs::path& inPath, const fs::path& outDirectoryPath, FileProgress& fileProgress,
             const bool &enable_lidar = true, const bool &enable_cam = false, const bool &enable_radar = false);

    private:
    void convertSampleDatas(rosbag::Bag& outBag, const fs::path &inPath, FileProgress& fileProgress,
                            const bool &enable_lidar = true, const bool &enable_cam = false, const bool &enable_radar = false);
    void convertEgoPoseInfos(rosbag::Bag& outBag, const std::vector<CalibratedSensorInfoAndName>& calibratedSensorInfo);
    void convertIMUDatas(rosbag::Bag& outBag);
    void convertWheelSpeedDatas(rosbag::Bag& outBag);
    
    private:
    const MetaDataProvider& metaDataProvider;
    const CANBusDataReader& canBusDataReader;
    std::vector<SampleDataInfo> sampleDatas;
    std::vector<EgoPoseInfo> egoPoseInfos;
    std::vector<IMUData> imuDatas;
    std::vector<WheelSpeedData> wheelSpeedDatas;

    SceneId sceneId;
    Token sceneToken;
};

}
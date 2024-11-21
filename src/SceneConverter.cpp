#include "nuscenes2bag/SceneConverter.hpp"
#include "nuscenes2bag/DatasetTypes.hpp"
#include "nuscenes2bag/utils.hpp"

#include "nuscenes2bag/EgoPoseConverter.hpp"
#include "nuscenes2bag/ImageDirectoryConverter.hpp"
#include "nuscenes2bag/LidarDirectoryConverter.hpp"
#include "nuscenes2bag/LidarDirectoryConverterXYZIR.hpp"
#include "nuscenes2bag/RadarDirectoryConverter.hpp"
#include "nuscenes2bag/IMUDataConverter.hpp"
#include "nuscenes2bag/VehInfoConverter.hpp"

#include <array>
#include <iostream>
#include <regex>
#include <string>

using namespace std;

namespace nuscenes2bag {

SceneConverter::SceneConverter(const MetaDataProvider& metaDataProvider, const CANBusDataReader& canBusDataReader)
  : metaDataProvider(metaDataProvider), canBusDataReader(canBusDataReader)
{}

boost::optional<SampleType>
getSampleType(const std::string& filename)
{
  std::array<std::pair<const char*, SampleType>, 3> pairs = {
    { { "CAM", SampleType::CAMERA },
      { "RADAR", SampleType::RADAR },
      { "LIDAR", SampleType::LIDAR } }
  };
  for (const auto& strAndSampleType : pairs) {
    const auto& str = strAndSampleType.first;
    const auto& sampleType = strAndSampleType.second;
    if (filename.find(str) != string::npos) {
      return boost::optional<SampleType>(sampleType);
    }
  }
  cout << "Unknown file " << filename << endl;
  return boost::none;
}

template<typename T>
void
writeMsg(const std::string topicName,
         const std::string& frameID,
         const TimeStamp timeStamp,
         rosbag::Bag& outBag,
         boost::optional<T> msgOpt)
{
  if (msgOpt) {
    auto& msg = msgOpt.value();
    msg.header.frame_id = frameID;
    msg.header.stamp = stampUs2RosTime(timeStamp);
    outBag.write(std::string(topicName).c_str(), msg.header.stamp, msg);
  }
}

static const std::regex TOPIC_REGEX = std::regex(".*__([A-Z_]+)__.*");

void
SceneConverter::submit(const Token& sceneToken, FileProgress& fileProgress)
{

  boost::optional<SceneInfo> sceneInfoOpt =
    metaDataProvider.getSceneInfo(sceneToken);
  // if(!sceneInfoOpt) {
  //     // cout << "SceneInfo for " << sceneToken << " not found!" << endl;
  //     return;
  // }
  assert(sceneInfoOpt);
  SceneInfo& sceneInfo = sceneInfoOpt.value();

  sceneId = sceneInfo.sceneId;
  this->sceneToken = sceneToken;
  sampleDatas = metaDataProvider.getSceneSampleData(sceneToken);
  egoPoseInfos = metaDataProvider.getEgoPoseInfo(sceneToken);
  imuDatas = canBusDataReader.getIMUData(sceneId);
  wheelSpeedDatas = canBusDataReader.getWheelSpeedData(sceneId);

  fileProgress.addToProcess(sampleDatas.size());
}

void
SceneConverter::run(const fs::path& inPath,
                    const fs::path& outDirectoryPath,
                    FileProgress& fileProgress,
                    const bool &enable_lidar,
                    const bool &enable_cam,
                    const bool &enable_radar)
{
  std::ostringstream number_str;
  number_str << std::setw(4) << std::setfill('0') << sceneId; 
  std::string bag_name_prefix = "scene-" + number_str.str();
  std::string bagName =
    outDirectoryPath.string() + "/" + bag_name_prefix + ".bag";

  rosbag::Bag outBag;
  outBag.open(bagName, rosbag::bagmode::Write);

  auto sensorInfos = metaDataProvider.getSceneCalibratedSensorInfo(sceneToken);
  convertEgoPoseInfos(outBag, sensorInfos);
  convertIMUDatas(outBag);
  convertWheelSpeedDatas(outBag);
  convertSampleDatas(outBag, inPath, fileProgress, enable_lidar, enable_cam, enable_radar);

  outBag.close();
}

void SceneConverter::convertIMUDatas(rosbag::Bag& outBag) {
  const std::string imuTopic = "/imu";
  for (const auto& imu_data : imuDatas) {
    sensor_msgs::Imu imu_msg = ImuData2SensorMsg(imu_data);
    outBag.write(imuTopic.c_str(), imu_msg.header.stamp, imu_msg);
  }
}

void SceneConverter::convertWheelSpeedDatas(rosbag::Bag& outBag) {
  const std::string twistTopic = "/wheel_speed";
  for (const auto& wheel_speed : wheelSpeedDatas) {
    geometry_msgs::TwistWithCovarianceStamped twist_msg = WheelSpeedData2TwistMsg(wheel_speed);
    outBag.write(twistTopic.c_str(), twist_msg.header.stamp, twist_msg);
  }
}

void
SceneConverter::convertSampleDatas(rosbag::Bag& outBag,
                                   const fs::path& inPath,
                                   FileProgress& fileProgress,
                                   const bool &enable_lidar,
                                   const bool &enable_cam,
                                   const bool &enable_radar)
{
  int lidar_seq_cnt = 0;
  for (const auto& sampleData : sampleDatas) {
    fs::path sampleFilePath = inPath / sampleData.fileName;

    boost::optional<SampleType> sampleTypeOpt =
      getSampleType(sampleFilePath.string());
    if (!sampleTypeOpt) {
      continue;
    }
    SampleType& sampleType = sampleTypeOpt.value();

    CalibratedSensorInfo calibratedSensorInfo =
      metaDataProvider.getCalibratedSensorInfo(
        sampleData.calibratedSensorToken);
    CalibratedSensorName calibratedSensorName =
      metaDataProvider.getSensorName(calibratedSensorInfo.sensorToken);
    std::string sensorName = toLower(calibratedSensorName.name);

    if (enable_cam && sampleType == SampleType::CAMERA) {
      auto topicName = "/" + sensorName + "/raw";
      auto msg = readImageFile(sampleFilePath);
      writeMsg(topicName, sensorName, sampleData.timeStamp, outBag, msg);

    } else if (enable_lidar && sampleType == SampleType::LIDAR) {
      auto topicName = "/" + sensorName;

      // PointCloud format:
      // auto msg = readLidarFile(sampleFilePath); // x,y,z,intensity
      auto msg = readLidarFileXYZIR(sampleFilePath); // x,y,z,intensity,ring
            
      writeMsg(topicName, sensorName, sampleData.timeStamp, outBag, msg);

    } else if (enable_radar && sampleType == SampleType::RADAR) {
      auto topicName = "/" + sensorName;
      auto msg = readRadarFile(sampleFilePath);
      writeMsg(topicName, sensorName, sampleData.timeStamp, outBag, msg);

    } else if(sampleType != SampleType::RADAR && sampleType != SampleType::LIDAR && sampleType != SampleType::CAMERA){
      cout << "Unknown sample type" << endl;
    }

    fileProgress.addToProcessed(1);
  }
}

geometry_msgs::TransformStamped
makeTransform(const char* frame_id,
              const char* child_frame_id,
              const double* translation,
              const double* rotation,
              ros::Time stamp = ros::Time(0))
{
  geometry_msgs::TransformStamped msg;
  msg.header.frame_id = std::string(frame_id);
  msg.header.stamp = stamp;
  msg.child_frame_id = std::string(child_frame_id);
  assignArray2Vector3(msg.transform.translation, translation);
  assignArray2Quaternion(msg.transform.rotation, rotation);
  return msg;
}

geometry_msgs::TransformStamped
makeIdentityTransform(const char* frame_id,
                      const char* child_frame_id,
                      ros::Time stamp = ros::Time(0))
{
  geometry_msgs::TransformStamped msg;
  msg.header.frame_id = std::string(frame_id);
  msg.header.stamp = stamp;
  msg.child_frame_id = std::string(child_frame_id);
  msg.transform.rotation.w = 1;
  return msg;
}

void
SceneConverter::convertEgoPoseInfos(
  rosbag::Bag& outBag,
  const std::vector<CalibratedSensorInfoAndName>& calibratedSensorInfos)
{

  std::vector<geometry_msgs::TransformStamped> constantTransforms;
  for (const auto& calibratedSensorInfo : calibratedSensorInfos) {
    auto sensorTransform =
      makeTransform("base_link",
                    toLower(calibratedSensorInfo.name.name).c_str(),
                    calibratedSensorInfo.info.translation,
                    calibratedSensorInfo.info.rotation);
    constantTransforms.push_back(sensorTransform);
  }
  geometry_msgs::TransformStamped tfMap2Odom =
    makeIdentityTransform("map", "odom");
  constantTransforms.push_back(tfMap2Odom);

  const std::string odomTopic = "/odom";
  for (const auto& egoPose : egoPoseInfos) {
    // write odom
    nav_msgs::Odometry odomMsg = egoPoseInfo2OdometryMsg(egoPose);
    outBag.write(odomTopic.c_str(), odomMsg.header.stamp, odomMsg);

    // write TFs
    // geometry_msgs::TransformStamped tfOdom2Base =
    //   egoPoseInfo2TransformStamped(egoPose);
    tf::tfMessage tfMsg;
    // tfMsg.transforms.push_back(tfOdom2Base);
    for (const auto& constantTransform : constantTransforms) {
      auto constantTransformWithNewStamp = constantTransform;
      constantTransformWithNewStamp.header.stamp = odomMsg.header.stamp;
      tfMsg.transforms.push_back(constantTransformWithNewStamp);
    }
    outBag.write("/tf", odomMsg.header.stamp, tfMsg);
  }
}

}
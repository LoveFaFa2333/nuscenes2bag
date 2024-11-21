#pragma once

#include <sensor_msgs/Imu.h>
#include <nuscenes2bag/CANBusDataTypes.hpp>

namespace nuscenes2bag {

sensor_msgs::Imu ImuData2SensorMsg(const IMUData& imu_data);

}

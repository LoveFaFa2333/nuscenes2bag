#pragma once

#include <geometry_msgs/TwistWithCovarianceStamped.h>
#include <nuscenes2bag/CANBusDataTypes.hpp>

namespace nuscenes2bag {

geometry_msgs::TwistWithCovarianceStamped WheelSpeedData2TwistMsg(const WheelSpeedData& imu_data);

}

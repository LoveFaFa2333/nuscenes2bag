#include <nuscenes2bag/VehInfoConverter.hpp>
#include <nuscenes2bag/utils.hpp>

namespace nuscenes2bag {

geometry_msgs::TwistWithCovarianceStamped WheelSpeedData2TwistMsg(const WheelSpeedData& wheel_speed_data) {
    geometry_msgs::TwistWithCovarianceStamped twist_msg;
    twist_msg.header.frame_id = "base_link";
    twist_msg.header.stamp = stampUs2RosTime(wheel_speed_data.utime);
    
    twist_msg.twist.twist.linear.x = wheel_speed_data.avg_vel;
    twist_msg.twist.twist.angular.z = wheel_speed_data.yaw_rate;

    return twist_msg;
}
}

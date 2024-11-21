#include <nuscenes2bag/IMUDataConverter.hpp>
#include <nuscenes2bag/utils.hpp>

namespace nuscenes2bag {

sensor_msgs::Imu ImuData2SensorMsg(const IMUData& imu_data) {
    sensor_msgs::Imu imu_msg;
    imu_msg.header.frame_id = "base_link";
    imu_msg.header.stamp = stampUs2RosTime(imu_data.utime);
    
    imu_msg.angular_velocity.x = imu_data.rotation_rate[0];
    imu_msg.angular_velocity.y = imu_data.rotation_rate[1];
    imu_msg.angular_velocity.z = imu_data.rotation_rate[2];

    imu_msg.linear_acceleration.x = imu_data.linear_accel[0];
    imu_msg.linear_acceleration.y = imu_data.linear_accel[1];
    imu_msg.linear_acceleration.z = imu_data.linear_accel[2];

    imu_msg.orientation.w = imu_data.q[0];
    imu_msg.orientation.x = imu_data.q[1];
    imu_msg.orientation.y = imu_data.q[2];
    imu_msg.orientation.z = imu_data.q[3];

    return imu_msg;
}
}

#pragma once 

#include <string>
#include <array>
#include <cmath>

namespace nuscenes2bag {

struct IMUData
{
    uint64_t utime;
    double linear_accel[3];
    double rotation_rate[3];
    double q[4];
};

struct WheelSpeedData
{   
    void CalVelAndYawRate() {
        avg_vel = (left_speed + right_speed) * 0.5 * (2 * M_PI * radius) / 60;               // m/s
        yaw_rate = (right_speed - left_speed) * 2 * M_PI / 60  * reat_track * 0.5;           // rad/s
    }

    uint64_t utime;
    double left_speed;
    double right_speed;
    double avg_vel;
    double yaw_rate;
    double radius = 0.305;      // Known Zoe wheel radius 0.305 m
    double reat_track = 1.510;  // Known Zoe rear track 1.510 m
};
}   // namespace nuscenes2bag
#pragma once

#include "point.hpp"
#include "path.hpp"
#include <vector>

namespace lib15442c
{
    class MotionProfileState : public PathState
    {
    public:
        MotionProfileState(float x, float y, float velocity, float acceleration, float jerk, float rotation, float rotation_speed, float look_ahead) :
            PathState(pos(x, y), rotation, velocity, look_ahead), acceleration(acceleration), jerk(jerk), rotation_speed(rotation_speed)
        {
        }

        float x;
        float y;
        float rotation;
        float velocity;
        float look_ahead;
        float acceleration;
        float jerk;
        float rotation_speed;
    };

    class MotionProfile : public Path
    {
    public:
        std::vector<MotionProfileState> path;
        MotionProfile(Point start_point, float start_yaw, float start_vel, float start_accel, Point end_point, float end_yaw, float end_vel, float end_accel, float max_vel, float max_accel, float maxJerk, float maxAngleChange, float look_ahead);

        MotionProfile(std::vector<Point> Points, std::vector<float> yaws, std::vector<float> velocities, std::vector<float> accelerations, std::vector<float> look_aheads, float max_velocity, float max_acceleration, float max_jerk, float max_angle_change);
    };
};
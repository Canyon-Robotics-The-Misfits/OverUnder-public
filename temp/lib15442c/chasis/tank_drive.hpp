#pragma once

#include "pros/imu.hpp"
#include "pros/motor_group.hpp"
#include "pros/motors.hpp"
#include "pros/rotation.hpp"
#include "pros/rtos.hpp"
#include <cmath>
#include <memory>

#include "lib15442c/chasis/drivetrain.hpp"
#include "lib15442c/chasis/odometry.hpp"
#include "lib15442c/controller/base_controllers.hpp"
#include "lib15442c/controller/pid.hpp"
#include "lib15442c/path/path.hpp"
#include "lib15442c/path/point.hpp"

namespace lib15442c
{
    class TankDrive: virtual public lib15442c::IDrivetrain
    {
    private:
        // Drivetrain Motors
        std::shared_ptr<pros::v5::MotorGroup> left_motors;
        std::shared_ptr<pros::v5::MotorGroup> right_motors;

        // Settings
        float track_width;
        float deg_inch_ratio;

    public:
        TankDrive(
            std::shared_ptr<pros::v5::MotorGroup> left_motors,
            std::shared_ptr<pros::v5::MotorGroup> right_motors,
            float wheel_diameter,
            float gear_ratio,
            float track_width);

        /**
         * @brief Move the robot with a forward/back speed and a rotational speed
         *
         * @param linear_speed The speed to travel forwards/backwards
         * @param rotational_speed The speed to rotate at
         */
        void move(float linear_speed, float rotational_speed);

        /**
         * @brief Move the robot with a forward/back speed and a rotational speed,
         * prioritizing the rotational speed over the forward/back speed
         *
         * @param linear_speed The speed to travel forwards/backwards
         * @param rotational_speed The speed to rotate at
         */
        void move_ratio(float linear_speed, float rotational_speed);

        /**
         * @brief Move the robot with a forward/back speed and a rotational speed
         *
         * @param linear_speed The speed to travel forwards/backwards in inches per
         * second
         * @param rotational_speed The speed to rotate at in degrees per second
         */
        void move_speed(float linear_speed, float rotational_speed);

        /**
         * @brief Move the left and right sides of the drivetrain individually
         *
         * @param left_speed The speed of the left side of the drivetrain
         * @param right_speed The speed of the right side of the drivetrain
         */
        void tank(float left_speed, float right_speed);

        /**
         * @brief Move the left and right sides of the drivetrain individually using the built-in PID
         *
         * @param left_speed The speed of the left side of the drivetrain
         * @param right_speed The speed of the right side of the drivetrain
         */
        void tank_pid(float left_speed, float right_speed);

        /**
         * @brief Set the brake mode of the drivetrain
         *
         * @param mode The brake mode to set
         */
        void set_brake_mode(pros::motor_brake_mode_e_t mode);

        float get_track_width();

        std::vector<const char*> get_temp_levels();
    };
} // namespace lib15442c
#pragma once

#include "pros/motors.hpp"

#include <cmath>

namespace lib15442c {
    class IDrivetrain
    {
    public:
        /**
         * @brief Move the robot with a forward/back speed and a rotational speed
         *
         * @param linear_speed The speed to travel forwards/backwards
         * @param rotational_speed The speed to rotate at
         */
        virtual void move(float linear_speed, float rotational_speed) = 0;

        /**
         * @brief Move the robot with a forward/back speed and a rotational speed,
         * prioritizing the rotational speed over the forward/back speed
         *
         * @param linear_speed The speed to travel forwards/backwards
         * @param rotational_speed The speed to rotate at
         */
        virtual void move_ratio(float linear_speed, float rotational_speed) = 0;

        /**
         * @brief Move the robot with a forward/back speed and a rotational speed
         *
         * @param linear_speed The speed to travel forwards/backwards in inches per
         * second
         * @param rotational_speed The speed to rotate at in degrees per second
         */
        virtual void move_speed(float linear_speed, float rotational_speed) = 0;

        /**
         * @brief Move the left and right sides of the drivetrain individually
         *
         * @param left_speed The speed of the left side of the drivetrain in volts (-127, 127)
         * @param right_speed The speed of the right side of the drivetrain in volts (-127, 127)
         */
        virtual void tank(float left_speed, float right_speed) = 0;

        /**
         * @brief Move the left and right sides of the drivetrain individually using the built-in PID
         *
         * @param left_speed The speed of the left side of the drivetrain in inches per second
         * @param right_speed The speed of the right side of the drivetrain in inches per second
         */
        virtual void tank_pid(float left_speed, float right_speed) = 0;

        /**
         * @brief Set the brake mode of the motors
         * 
         * @param mode 
         */
        virtual void set_brake_mode(pros::motor_brake_mode_e_t mode) = 0;

        /**
         * @brief Get the distance between the wheels
         * 
         * @return float 
         */
        virtual float get_track_width() = 0;

        /**
         * @brief Get the temp levels of the motors
         * 
         * @return std::vector<const char*> 
         */
        virtual std::vector<const char*>  get_temp_levels() = 0;
    };
}
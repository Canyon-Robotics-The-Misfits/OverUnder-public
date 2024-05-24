#pragma once

#include "lib15442c/path/point.hpp"
#include "pros/rotation.hpp"
#include "pros/gps.hpp"
#include "pros/imu.hpp"
#include <memory>

namespace lib15442c
{
    class IOdometry
    {
    public:
        /**
         * @brief Get the x position of the robot
        */
        virtual float getX() = 0;
        /**
         * @brief Get the y position of the robot
        */
        virtual float getY() = 0;
        /**
         * @brief Get the position of the robot
        */
        virtual Point getPosition() = 0;

        /**
         * @brief Set the x position of the robot
         * 
         * @param x The new x position
        */
        virtual void setX(float x) = 0;
        /**
         * @brief Set the y position of the robot
         * 
         * @param y The new y position
        */
        virtual void setY(float y) = 0;
        /**
         * @brief Set the position of the robot
         * 
         * @param position The new position
        */
        virtual void setPosition(Point position) = 0;

        /**
         * @brief Get the rotation of the robot
        */
        virtual float getRotation() = 0;

        /**
         * @brief Get the rotation of the robot using the second inertial sensor
         * 
         */
        virtual float getRotation2() = 0;

        /**
         * @brief Set the rotation of the robot
         * 
         * @param rotation The new rotation of the robot
        */
        virtual void setRotation(float rotation) = 0;
    };

    class TrackerOdom : public virtual IOdometry
    {
    private:
        // Sensor
        std::shared_ptr<pros::Rotation> parallel_tracker;
        std::shared_ptr<pros::Rotation> perpendicular_tracker;
        std::shared_ptr<pros::v5::IMU> inertial;

        // Settings
        float inertial_scale;
        float tracker_circumfrance;
        float perpendicular_tracker_offset;
        float parallel_tracker_offset;

        // Inertial 2
        std::shared_ptr<pros::v5::IMU> inertial_2;
        float inertial_scale_2;

        // Rotation Offset
        float rotation_offset = 0;

        // Robot position
        pros::rtos::Mutex position_mutex = pros::rtos::Mutex();
        Point position = Point(0, 0);

        // Tasks
        pros::Task task = pros::Task([] { return; });

    public:
        TrackerOdom(
            std::shared_ptr<pros::Rotation> parallel_tracker,
            std::shared_ptr<pros::Rotation> perpendicular_tracker,
            std::shared_ptr<pros::IMU> inertial, float inertial_scale,
            float tracker_circumfrance, float perpendicular_tracker_offset, float parallel_tracker_offset, 
            std::shared_ptr<pros::IMU> inertial_2 = NULL, float inertial_scale_2 = 0) :

                parallel_tracker(parallel_tracker), perpendicular_tracker(perpendicular_tracker),
                inertial(inertial), inertial_scale(inertial_scale), tracker_circumfrance(tracker_circumfrance),
                perpendicular_tracker_offset(perpendicular_tracker_offset), parallel_tracker_offset(parallel_tracker_offset),
                inertial_2(inertial_2), inertial_scale_2(inertial_scale_2) {};
        
        ~TrackerOdom();

        float getX();
        float getY();
        Point getPosition();

        void setX(float x);
        void setY(float y);
        void setPosition(Point position);

        float getRotation();
        float getRotation2();
        void setRotation(float rotation);

        /**
         * @brief Start the odometry background task
        */
        void startTask();
        /**
         * @brief Stop the odometry background task
        */
        void stopTask();
    };

    class GPSOdom : public virtual IOdometry {
    private:
        pros::GPS gps;

        float rotation_offset;

        static constexpr float inches_per_meter = 39.3701;

    public:
        GPSOdom(int port, float x_offset, float y_offset, float rotation_offset);

        float getX();
        float getY();
        Point getPosition();

        void setX(float x);
        void setY(float y);
        void setPosition(Point position);

        float getRotation();
        void setRotation(float rotation);
    };
}
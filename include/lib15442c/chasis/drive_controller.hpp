#pragma once

#include "lib15442c/path/point.hpp"
#include "lib15442c/path/path.hpp"
#include "lib15442c/controller/pid.hpp"
#include "drivetrain.hpp"
#include "odometry.hpp"

#include <cmath>
#include <memory>

namespace lib15442c
{
    class DriveController
    {
    private:
        std::shared_ptr<lib15442c::IDrivetrain> drivetrain;
        std::shared_ptr<lib15442c::IOdometry> odometry;

        std::shared_ptr<lib15442c::PID> drive_controller;
        std::shared_ptr<lib15442c::PID> turn_controller;

        pros::Mutex async_mutex;

        float default_min_speed = 12;
        float default_max_speed = 127;
        float default_threshold = NAN;
        int default_timeout = 1500;
        bool default_async = false;
        bool default_chain = false;

        float min_speed = default_min_speed;
        float max_speed = default_max_speed;
        float threshold = default_threshold;
        int timeout = default_timeout;
        bool async = default_async;
        bool chain = default_chain;

        // Stop all tasks that are running
        pros::Mutex stop_mutex;
        bool stopFlag = false;
        bool shouldStop();

    public:
        DriveController(
            std::shared_ptr<lib15442c::IDrivetrain> drivetrain,
            std::shared_ptr<lib15442c::IOdometry> odometry,
            std::shared_ptr<lib15442c::PID> drive_pid,
            std::shared_ptr<lib15442c::PID> turn_pid);

        /**
         * @brief Stop all tasks that are running
         * 
         * @param timeout The amount of time to wait before stopping all tasks
         */
        void stopAllTasks(float timeout = 100);

        /**
         * @brief Drive a certain distance
         *
         * @param distance The distance to drive (inches)
         * @param angle The angle to drive at (degrees)
         */
        void drive(float distance, float angle = INFINITY);
        
        // Arc direction enum parameter
        enum class ArcDirection
        {
            Forward = 1,
            Backward = -1
        };

        /**
         * @brief Face an angle based on the robot's current rotation
         *
         * @param angle The amount to turn (degrees)
         * @param offset The amount the center of rotation is offset by
         * @param arcDirection The direction of the arc
         */
        void turn(float angle, float offset = 0, ArcDirection arcDirection = ArcDirection::Forward);

        /**
         * @brief Face a point in (x, y) coordinates
         *
         * @param point The point to face
         * @param angle_offset The amount to offset the angle by
         * @param offset The amount the center of rotation is offset by
         * @param arcDirection The direction of the arc
         */
        void facePoint(lib15442c::Point point, float angle_offset = 0,
                       float offset = 0, ArcDirection arcDirection = ArcDirection::Forward);

        /**
         * @brief Face an angle based on the rotation of the field
         *
         * @param angle The angle to turn to (degrees)
         * @param offset The amount the center of rotation is offset by
         * @param arcDirection The direction of the arc
         */
        void faceAngle(float angle, float offset = 0, ArcDirection arcDirection = ArcDirection::Forward, Point targetPoint = pos(INFINITY, INFINITY), float angleOffset = 0);

        /**
         * @brief Drive to a Point using the Boomerang Controller
         *
         * @param point The point to drive to
         * @param angle The target angle to end at. If INFINITY, do not try to end at an angle
         * @param backwards Whether to go backwards
         * @param lead The lead to use for the boomerange controller
         * @param cross_track_threshold Prioritize the angle within this threshold
         * @param turn_priority How much to prioritize turning at the begining of the drive
         * @param turn_dampening How much to dampen the turn pid
         */
        void boomerang(lib15442c::Point point, float angle = INFINITY, bool backwards = false, float lead = 0.6, float cross_track_threshold = 3, float turn_priority = 15, float turn_dampening = 0.6);

        /**
         * @brief Follow a path with pure pursuit
         *
         * @param path The path to follow
         * @param backwards Whether to drive backwards
         * @param async Whether to do the movement asynchronously
         */
        void followPath(lib15442c::Path path, bool backwards = false,
                        bool async = false);

        ///////////////////////////// PARAMETERS /////////////////////////////

        /**
         * @brief Reset the parameters
        */
        void resetParams();

        /**
         * @brief Set the min speed of the next movement
        */
        void setMinSpeed(float minSpeed);
        
        /**
         * @brief Set the max speed of the next movement
        */
        void setMaxSpeed(float maxSpeed);

        /**
         * @brief Set the threshold of the next movement
        */
        void setThreshold(float threshold);

        /**
         * @brief Set the timeout of the next movement
        */
        void setTimeout(float timeout);

        /**
         * @brief Set whether the next movement is async
        */
        void setAsync(bool async);
        /**
         * @brief Whether to chain the next move into the one after
        */
        void setChained(bool chained);

        ///////////////////////////// ASYNC CONTROLS /////////////////////////////

        /**
         * @brief Whether the robot is done moving
         *
         * @param timeout How long to wait before giving up on waiting
         *
         * @return true The robot is done moving
         * @return false The robot is not done moving
         */
        bool isDone(int timeout = 0);

        /**
         * @brief Wait for the robot to finish it's current motion
         */
        void awaitDone();

        /**
         * @brief Wait for the robot to reach a certain point, or for the robot's
         * current motion to end
         *
         * @param point The point to wait until the robot is near
         * @param distance The distance the robot must be closer then to the point
         * @param checkForDone Whether to check if the robot is done moving
         *
         * @return bool Whether the wait ended at the right time or at the end of
         * the movement
         */
        bool awaitNear(lib15442c::Point point, float distance = 1, bool checkForDone = true);

        /**
         * @brief Wait for the robot to be close to a certain angle, or for the
         * robot's current motion to end
         *
         * @param angle The angle to wait until the robot is close to
         * @param threshold How close the robot needs to be to the right angle
         * @param checkForDone Whether to check if the robot is done moving
         *
         * @return bool Whether the wait ended at the right time or at the end of
         * the movement
         */
        bool awaitAngle(float angle, float threshold = 2, bool checkForDone = true);
    };
}
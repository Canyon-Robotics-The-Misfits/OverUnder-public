#include "lib15442c/chasis/odometry.hpp"
#include "lib15442c/path/point.hpp"
#include "lib15442c/util/angles.hpp"
#include "lib15442c/logger.hpp"

#include <math.h>

#define LOGGER "Odometry"

// Tracker Odom

lib15442c::TrackerOdom::~TrackerOdom()
{
    stopTask();
}

void lib15442c::TrackerOdom::setX(float val)
{
    position_mutex.lock();
    position.x = val;
    position_mutex.unlock();
}

void lib15442c::TrackerOdom::setY(float val)
{
    position_mutex.lock();
    position.y = val;
    position_mutex.unlock();
}

float lib15442c::TrackerOdom::getX()
{
    position_mutex.lock();
    float temp = position.x;
    position_mutex.unlock();

    return temp;
}

float lib15442c::TrackerOdom::getY()
{
    position_mutex.lock();
    float temp = position.y;
    position_mutex.unlock();

    return temp;
}

lib15442c::Point lib15442c::TrackerOdom::getPosition()
{
    position_mutex.lock();
    Point temp = position;
    position_mutex.unlock();

    return temp;
}

void lib15442c::TrackerOdom::setPosition(lib15442c::Point position)
{
    position_mutex.lock();
    this->position = position;
    position_mutex.unlock();
}

float lib15442c::TrackerOdom::getRotation()
{
    position_mutex.lock();
    float imu_1 = inertial->get_rotation() * inertial_scale;
    // float imu_2 = imu_1;

    // if (inertial_scale_2 != 0) {
    //     imu_2 = inertial_2->get_rotation() * inertial_scale_2;
    // }
    position_mutex.unlock();

    // return (imu_1 + imu_2) / 2.0;
    return imu_1;
}

float lib15442c::TrackerOdom::getRotation2()
{
    position_mutex.lock();
    float imu_2 = inertial_2->get_rotation();
    position_mutex.unlock();

    return imu_2;
}

void lib15442c::TrackerOdom::setRotation(float rotationOffset)
{
    position_mutex.lock();
    rotation_offset = rotationOffset;
    position_mutex.unlock();
    // Makes sure odometry updates before continuing
    pros::delay(15);
}

void lib15442c::TrackerOdom::startTask()
{

    task = pros::Task([this]
                      {
        inertial->tare();
        // if (inertial_scale_2 != 0) {
        //     inertial_2->tare();
        // }
        parallel_tracker->reset_position();
        perpendicular_tracker->reset_position();

        float last_parallel = parallel_tracker->get_position() / 100.0;
        float last_perpendicular = perpendicular_tracker->get_position() / 100.0;
        // last_perpendicular = 0;

        int time = 0;
        int tickTimer = 0;

        float offset_zero = 0;
        float last_angle = getRotation();

        float degrees_per_inch = tracker_circumfrance / 360;

        while (true)
        {

            if (rotation_offset != 0)
            {
                inertial->set_rotation(rotation_offset / inertial_scale);
                // if (inertial_scale_2 != 0) {
                //     inertial_2->set_rotation(rotation_offset / inertial_scale_2);
                // }
                offset_zero = rotation_offset;
                rotation_offset = 0;
            }
            else
            {
                // Get the tracker wheel encoder positions
                float parallel = parallel_tracker->get_position() / 100.0;
                float perpendicular = perpendicular_tracker->get_position() / 100.0;
                // perpendicular = 0;

                // Get the current robot rotation
                float angle = getRotation();

                if (isnan(parallel) || isnan(perpendicular) || isnan(angle))
                {
                    ERROR("Parallel, perpendicular, or angle is NaN, skipping this iteration. %f, %f, %f", parallel, perpendicular, angle);
                    continue;
                }

                // Modify the encoders to compensate for turning              
                parallel -= (angle - offset_zero) * perpendicular_tracker_offset;
                perpendicular -= (angle - offset_zero) * parallel_tracker_offset;

                // std::cout << angle << ", " << parallel << ", " << perpendicular << std::endl;

                // Calculate the change in the horizontal and vertical encoder
                float deltaTheta = angle - last_angle;
                float deltaParallel = (parallel - last_parallel) * degrees_per_inch;
                float deltaPerpendicular = (perpendicular - last_perpendicular) * degrees_per_inch;

                // std::cout << deltaTheta << ", " << deltaParallel << ", " << deltaPerpendicular << std::endl;

                position_mutex.lock();

                if (deltaTheta == 0)
                {
                    position += pos(
                        sin(rad(angle)) * deltaParallel +
                            sin(rad(angle + 90)) * deltaPerpendicular,
                        cos(rad(angle)) * deltaParallel +
                            cos(rad(angle + 90)) * deltaPerpendicular);
                }
                else
                {
                    float radiusParallel = deltaParallel / rad(deltaTheta);
                    float radiusPerpendicular = deltaPerpendicular / rad(deltaTheta);

                    float delta_x = (cos(rad(angle)) - cos(rad(last_angle))) * radiusParallel;
                    delta_x += (cos(rad(angle + 90)) - cos(rad(last_angle + 90))) * radiusPerpendicular;

                    float delta_y = (sin(rad(angle)) - sin(rad(last_angle))) * radiusParallel;
                    delta_y += (sin(rad(angle + 90)) - sin(rad(last_angle + 90))) * radiusPerpendicular;

                    position += pos(
                        -delta_x,
                        delta_y
                    );
                }
        
                if (isnan(position.x)) {
                    ERROR_TEXT("X Position is NaN, resetting it");
                    position.x = 0;
                }
                if (isnan(position.y)) {
                    ERROR_TEXT("Y Position is NaN, resetting it");
                    position.y = 0;
                }

                // Log position in terminal
                // if (tickTimer % 10 == 0)
                //     std::cout << position.x << ", " << position.y << std::endl;

                position_mutex.unlock(); // unlock the mutex

                // Set the last variables
                if(!isnan(perpendicular) && !isnan(last_parallel) && !isnan(angle)) {
                    last_perpendicular = perpendicular;
                    last_parallel = parallel;
                    last_angle = angle;
                }
                
            }

            if (pros::Task::notify_take(true, 10) > 0 || ((pros::c::competition_get_status() & COMPETITION_DISABLED) != 0)) {
                break;
            }
            time += 10;
            tickTimer++;
        } });
}

void lib15442c::TrackerOdom::stopTask()
{
    task.notify();
    task.join();
}

// GPS odom

lib15442c::GPSOdom::GPSOdom(int port, float x_offset, float y_offset, float rotation_offset) : gps(pros::GPS(port)), rotation_offset(rotation_offset)
{
    gps.set_offset(x_offset, y_offset);
};

void lib15442c::GPSOdom::setX(float val)
{
    gps.set_position((val - 72) / inches_per_meter, (getY() - 72) / inches_per_meter, getRotation() - rotation_offset);
}

void lib15442c::GPSOdom::setY(float val)
{
    gps.set_position((getX() - 72) / inches_per_meter, (val - 72) / inches_per_meter, getRotation() - rotation_offset);
}

float lib15442c::GPSOdom::getX()
{
    return gps.get_position().x * inches_per_meter + 72;
}

float lib15442c::GPSOdom::getY()
{
    return gps.get_position().y * inches_per_meter + 72;
}

lib15442c::Point lib15442c::GPSOdom::getPosition()
{
    auto gps_position = gps.get_position();
    lib15442c::Point position = pos(gps_position.x + 72 / inches_per_meter, gps_position.y + 72 / inches_per_meter);

    return position * inches_per_meter;
}

void lib15442c::GPSOdom::setPosition(lib15442c::Point position)
{
    gps.set_position((position.x - 72) / inches_per_meter, (position.y - 72) / inches_per_meter, getRotation() - rotation_offset);
}

float lib15442c::GPSOdom::getRotation()
{
    return gps.get_rotation() + rotation_offset;
}

void lib15442c::GPSOdom::setRotation(float rotation)
{
    gps.set_rotation(rotation - rotation_offset);
}
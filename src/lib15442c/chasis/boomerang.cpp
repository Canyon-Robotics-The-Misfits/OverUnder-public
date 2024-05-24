#include "lib15442c/chasis/drive_controller.hpp"
#include "lib15442c/path/point.hpp"
#include "lib15442c/util/angles.hpp"
#include "lib15442c/util/math.hpp"

#include "lib15442c/logger.hpp"

#define LOGGER "Boomerang"

void lib15442c::DriveController::boomerang(lib15442c::Point point, float target_angle, bool backwards, float lead, float cross_track_threshold, float turn_priority, float turn_dampening)
{
    if (async)
    {
        async = false;
        
        pros::Task([this, point, target_angle, backwards, lead, cross_track_threshold, turn_priority, turn_dampening]
                   { boomerang(point, target_angle, backwards, lead, cross_track_threshold, turn_priority, turn_dampening); });
        pros::delay(10);
        return;
    }
    async_mutex.lock();
    
    INFO("Start boomerang... (%f, %f)", point.x, point.y);

    drive_controller->reset();
    turn_controller->reset();

    int startingTime = pros::millis();

    auto comp_status = pros::competition::get_status();

    target_angle = rad(target_angle + (180 * backwards));

    Point position = odometry->getPosition();
    bool initial_above_approach_line = position.y > tan(target_angle) * (position.x - point.x) + point.y;

    while (pros::competition::get_status() == comp_status)
    {
        Point position = odometry->getPosition();
        float error = distance_between(position, point);

        Point caret = point;

        if (target_angle != INFINITY)
        {
            caret -= pos(cos(-target_angle + PI / 2.0), sin(-target_angle + PI / 2.0)) * lead * error;
        }

        float carret_angle = angle_between(position, caret) + (180 * backwards);
        if (caret.y - position.y < 0)
        {
            carret_angle -= 180;
        }

        // If it is close to the end, focus on getting to the right angle and use cross track error
        if (error < cross_track_threshold && target_angle != INFINITY)
        {
            carret_angle = deg(target_angle);
            float newTargetX = odometry->getY() + caret.x * tan(rad(odometry->getRotation() + 90)) - odometry->getX() * tan(rad(odometry->getRotation())) - caret.y;
            newTargetX /= tan(rad(odometry->getRotation() + 90)) - tan(rad(odometry->getRotation()));
            float newTargetY = odometry->getY() + (newTargetX - odometry->getX()) * tan(rad(odometry->getRotation()));
            // float temp = error;
            error = distance_between(pos(newTargetX, newTargetY), point);
            // std::cout << temp << ", " << error << std::endl;
        }

        bool above_approach_line = position.y > tan(target_angle) * (position.x - point.x) + point.y;
        if (this->chain && fabs(error) < (isnan(this->threshold) ? 5 : this->threshold) && initial_above_approach_line != above_approach_line) {
            break;
        }
        else if (this->chain && fabs(error) < 1)
        {
            break;
        }
        else if (!this->chain && fabs(error) < (isnan(this->threshold) ? 1 : this->threshold))
        {
            break;
        }

        if ((int)pros::millis() - startingTime >= this->timeout)
        {
            WARN_TEXT("boomerang timed out!");
            break;
        }

        // End the task if stopFlag is set
        if (stopFlag)
        {
            INFO_TEXT("Stopping boomerang!");
            break;
        }

        float speed = drive_controller->calculateError(error) * (backwards ? -1 : 1);

        // speed += std::fmin(fabs(totalError) * 0.34, 30.0) * lib15442c::sgn(error);

        float angle_error = lib15442c::angle_error(odometry->getRotation(), carret_angle);
        float rot_speed = -turn_controller->calculateError(angle_error) * (error < 2 ? error / 2.0 : 1.0); // NOTE: remove <2 /2 for testing

        speed = std::fmax(fabs(speed), 18) * lib15442c::sgn(speed);

        if (abs(speed) > max_speed)
        {
            speed = max_speed * lib15442c::sgn(speed);
        }

        if (abs(rot_speed) > 127)
        {
            rot_speed = 127 * lib15442c::sgn(rot_speed);
        }

        // Soften the turn speed
        rot_speed = std::min(fabs(rot_speed), fabs(speed) * turn_dampening) * lib15442c::sgn(rot_speed);

        // Multiple the speed on a scale of 0-1 based on the angle error
        if (angle_error != 0 && turn_priority != -1)
        {
            speed *= fmax(fmin(fabs(turn_priority / fabs(angle_error)), 1), 0);
        }

        if (abs(speed) < min_speed)
        {
            speed = min_speed * lib15442c::sgn(speed);
        }

        // std::cout << speed << std::endl;

        std::cout << odometry->getX() << ", " << odometry->getY() << ", " << caret.x << ", " << caret.y << std::endl;

        drivetrain->move_ratio(speed, rot_speed);

        pros::delay(20);
    }

    if (!chain)
    {
        drivetrain->move(0, 0);
    }

    resetParams();

    async_mutex.unlock();
    
    INFO_TEXT("End boomerang!");
}
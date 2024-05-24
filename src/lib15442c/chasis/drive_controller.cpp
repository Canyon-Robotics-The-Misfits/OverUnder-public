#include "lib15442c/chasis/drive_controller.hpp"
#include "lib15442c/path/point.hpp"
#include "lib15442c/util/angles.hpp"
#include "lib15442c/util/math.hpp"

#include "lib15442c/logger.hpp"

#define LOGGER "Drive Controller"

lib15442c::DriveController::DriveController(
    std::shared_ptr<lib15442c::IDrivetrain> drivetrain,
    std::shared_ptr<lib15442c::IOdometry> odometry,
    std::shared_ptr<lib15442c::PID> drive_pid,
    std::shared_ptr<lib15442c::PID> turn_pid)
    : drivetrain(drivetrain), odometry(odometry), drive_controller(drive_pid), turn_controller(turn_pid){};

void lib15442c::DriveController::stopAllTasks(float timeout)
{
    stop_mutex.lock();
    stopFlag = true;
    stop_mutex.unlock();

    int startTime = pros::millis();

    while (pros::millis() - startTime < timeout)
    {
        stop_mutex.lock();
        if (!stopFlag)
        {
            stop_mutex.unlock();
            return;
        }
        stop_mutex.unlock();
        pros::delay(20);
    }

    stop_mutex.lock();
    stopFlag = false;
    stop_mutex.unlock();

    ERROR_TEXT("Stopping all tasks timed out!");
}

bool lib15442c::DriveController::shouldStop()
{
    stop_mutex.lock();
    bool val = stopFlag;
    if (val)
    {
        stopFlag = false;
    }
    stop_mutex.unlock();
    return val;
}

void lib15442c::DriveController::turn(float angle, float offset, ArcDirection arcDirection)
{
    float global_angle = odometry->getRotation() + angle;

    faceAngle(global_angle, offset, arcDirection);
}

void lib15442c::DriveController::facePoint(lib15442c::Point point, float angle_offset, float offset, ArcDirection arcDirection)
{
    faceAngle(INFINITY, offset, arcDirection, point, angle_offset);
}

void lib15442c::DriveController::faceAngle(float targetAngle, float offset, ArcDirection arcDirection, Point targetPoint, float angleOffset)
{
    if (async)
    {
        async = false;

        pros::Task([this, targetAngle, offset, arcDirection]
                   { faceAngle(targetAngle, offset, arcDirection); });
        pros::delay(10);
        return;
    }
    async_mutex.lock();

    if (targetPoint.x == INFINITY || targetPoint.y == INFINITY)
    {
        INFO("Start Face Angle... (%f deg)", targetAngle);
    }
    else
    {
        INFO("Start Face Point... (%f, %f)", targetPoint.x, targetPoint.y);
    }

    // Calculate target angle if facing a point
    if (targetPoint.x != INFINITY && targetPoint.y != INFINITY)
    {
        targetAngle = lib15442c::angle_between(odometry->getPosition(), targetPoint);
        if (targetPoint.y - odometry->getY() < 0)
        {
            targetAngle += 180;
        }
        targetAngle += angleOffset;
    }

    // Calculate ratios
    float startRotation = odometry->getRotation();
    float initialError = angle_error(targetAngle, startRotation);

    float leftArcDistance = (offset + drivetrain->get_track_width() / 2.0) * rad(angle_error(-startRotation, targetAngle));
    float rightArcDistance = (offset - drivetrain->get_track_width() / 2.0) * rad(angle_error(-startRotation, targetAngle));

    float leftRatio = sgn(leftArcDistance) * fabs(fmax(fmin(leftArcDistance / rightArcDistance, 1), -1));
    float rightRatio = sgn(rightArcDistance) * fabs(fmax(fmin(rightArcDistance / leftArcDistance, 1), -1));

    // NOTE: This may need to be deleted (might not work)
    if (wrap_angle_180_180(startRotation) < wrap_angle_180_180(targetAngle))
    {
        leftRatio *= -1;
        rightRatio *= -1;
    }

    if (offset == 0)
    {
        leftRatio = 1;
        rightRatio = -1;
    }

    turn_controller->reset();
    int timeCorrect = 0;

    int startingTime = pros::millis();

    auto comp_status = pros::competition::get_status();

    while (pros::competition::get_status() == comp_status)
    {
        // Calculate target angle if facing a point
        if (targetPoint.x != INFINITY && targetPoint.y != INFINITY)
        {
            targetAngle = lib15442c::angle_between(odometry->getPosition(), targetPoint);
            if (targetPoint.y - odometry->getY() < 0)
            {
                targetAngle += 180;
            }
            targetAngle += angleOffset;
        }

        float rotation = odometry->getRotation();
        float error = lib15442c::angle_error(targetAngle, rotation);

        float turnThreshold = isnan(this->threshold) ? 1.5 : this->threshold;
        if (fabs(error) < (turnThreshold))
        {
            timeCorrect += 20;
        }
        else
        {
            timeCorrect = 0;
        }

        bool chainCondition = chain && (sgn(error) != sgn(initialError) || fabs(error) < 10 /*isnan(this->threshold) ? 10 : this->threshold*/);
        if (timeCorrect >= 100 || chainCondition)
        {
            // std::cout << pros::millis() << ", " << rotation << ", " << targetAngle << ", " << error << std::endl;
            break;
        }

        if ((int)pros::millis() - startingTime >= timeout)
        {
            WARN_TEXT("faceAngle timed out!");
            break;
        }

        // End the task if stopFlag is set
        if (shouldStop())
        {
            INFO_TEXT("Stopping faceAngle!");
            break;
        }

        float rot_speed = turn_controller->calculateError(error);

        // std::cout << pros::millis() << ", " << error << std::endl;

        rot_speed = std::max(min_speed, (float)fabs(rot_speed)) * lib15442c::sgn(rot_speed);
        rot_speed = std::min(max_speed, (float)fabs(rot_speed)) * lib15442c::sgn(rot_speed);
        if (fabs(error) < 2.5)
        {
            rot_speed = min_speed * lib15442c::sgn(error);
        }

        // std::cout << pros::millis() << ", " << rot_speed << ", " << error << std::endl;

        float leftSpeed = rot_speed * leftRatio;
        float rightSpeed = rot_speed * rightRatio;

        // // Adjust speeds to go in arc direction until close to target
        // if (fabs(error) < 20)
        // {
        //     if (arcDirection == ArcDirection::Forward)
        //     {
        //         leftSpeed = fabs(leftSpeed);
        //         rightSpeed = fabs(rightSpeed);
        //     }
        //     else if (arcDirection == ArcDirection::Backward)
        //     {
        //         leftSpeed = -fabs(leftSpeed);
        //         rightSpeed = -fabs(rightSpeed);
        //     }
        // }

        drivetrain->tank(leftSpeed, rightSpeed);

        pros::delay(20);
    }

    resetParams();

    if (!chain)
    {
        drivetrain->move(0, 0);
    }
    async_mutex.unlock();

    INFO_TEXT("End Face Angle!");
}

void lib15442c::DriveController::drive(float distance, float angle)
{
    if (async)
    {
        async = false;

        pros::Task([this, distance, angle]
                   { drive(distance, angle); });
        pros::delay(10);
        return;
    }
    async_mutex.lock();

    INFO("Start Drive... (%f in)", distance);

    drive_controller->reset();
    turn_controller->reset();

    Point starting_position = odometry->getPosition();

    float timeCorrect = 0;
    float totalError = 0;

    int startingTime = pros::millis();

    float target_rotation = angle;
    if (angle == INFINITY)
    {
        target_rotation = odometry->getRotation();
    }

    auto comp_status = pros::competition::get_status();

    while (pros::competition::get_status() == comp_status)
    {
        // float left = left_motors->get_position() - left_start;
        // float right = right_motors->get_position() - right_start;
        // float parallel = parallel_tracker->get_position() / 100.0;
        // float parallelDistance = (parallel - starting_parallel) * degrees_per_inch;

        float distanceTraveled = distance_between(starting_position, odometry->getPosition());
        float error = distance - (distanceTraveled * lib15442c::sgn(distance));

        float speed = drive_controller->calculateError(error);

        speed = fmin(fabs(speed), max_speed) * lib15442c::sgn(speed);
        speed = fmax(fabs(speed), min_speed) * lib15442c::sgn(speed);

        if (fabs(speed) < 40)
        {
            totalError += error;
        }

        speed += std::fmin(fabs(totalError) * 0.34, 30.0) * lib15442c::sgn(error);

        float angle_error = lib15442c::angle_error(target_rotation, odometry->getRotation());
        float rot_speed =
            // 0;
            turn_controller->calculateError(angle_error) * 0.25;

        // std::cout << pros::millis() << ", " << error << ", " << speed << std::endl;

        speed = std::fmax(fabs(speed), this->min_speed) * lib15442c::sgn(speed);

        // std::cout << pros::millis() << ", " << error << ", " << speed << std::endl;

        drivetrain->move(speed, rot_speed);

        float driveThreshold = isnan(this->threshold) ? 0.1 : this->threshold;
        if (!chain && fabs(error) < driveThreshold)
        {
            timeCorrect += 20;
            break;
        }
        else
        {
            timeCorrect = 0;
        }

        if (timeCorrect >= 40 || (chain && fabs(distanceTraveled) >= fabs(distance) - driveThreshold))
        {
            break;
        }

        if ((int)pros::millis() - startingTime >= this->timeout)
        {
            WARN_TEXT("drive timed out!");
            break;
        }

        // End the task if stopFlag is set
        if (shouldStop())
        {
            INFO_TEXT("Stopping drive!");
            break;
        }

        pros::delay(20);
    }

    resetParams();

    if (!chain)
    {
        drivetrain->move(0, 0);
    }
    async_mutex.unlock();

    INFO_TEXT("End Drive!");
}

void lib15442c::DriveController::resetParams()
{
    min_speed = default_min_speed;
    max_speed = default_max_speed;
    threshold = default_threshold;
    timeout = default_timeout;
    async = default_async;
    chain = default_chain;
}

void lib15442c::DriveController::setMinSpeed(float min_speed)
{
    this->min_speed = min_speed;
}
void lib15442c::DriveController::setMaxSpeed(float max_speed)
{
    this->max_speed = max_speed;
}

void lib15442c::DriveController::setThreshold(float threshold)
{
    this->threshold = threshold;
}

void lib15442c::DriveController::setTimeout(float timeout)
{
    this->timeout = timeout;
}

void lib15442c::DriveController::setAsync(bool async)
{
    this->async = async;
}
void lib15442c::DriveController::setChained(bool chained)
{
    this->chain = chained;
}

bool lib15442c::DriveController::isDone(int timeout)
{
    bool done = async_mutex.try_lock_for(std::chrono::milliseconds(timeout));
    if (done)
    {
        async_mutex.unlock();
    }

    return done;
}

void lib15442c::DriveController::awaitDone()
{
    async_mutex.take(TIMEOUT_MAX);
    async_mutex.unlock();
}

bool lib15442c::DriveController::awaitNear(lib15442c::Point point, float distance, bool checkForDone)
{
    while (true)
    {
        if (distance_between(point, odometry->getPosition()) < distance)
        {
            return true;
        }

        if (checkForDone && isDone(5))
        {
            return false;
        }
    }
}

bool lib15442c::DriveController::awaitAngle(float angle, float threshold, bool checkForDone)
{
    int startingTime = pros::millis();
    while (pros::millis() - startingTime < 2000)
    {
        float rotation = odometry->getRotation();

        if (fabs(lib15442c::angle_error(rotation, angle)) <= threshold)
        {
            return true;
        }

        if (checkForDone && isDone(5))
        {
            return false;
        }
    }
}
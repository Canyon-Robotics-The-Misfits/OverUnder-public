#include <memory>

#include "config.hpp"
#include "lib15442c/controller/pid.hpp"
#include "lib15442c/util/angles.hpp"
#include "lib15442c/chasis/tank_drive.hpp"
#include "main.h"

#define LOGGER "Robot Init"

std::shared_ptr<lib15442c::TrackerOdom> create_tracker_odometry()
{
    INFO_TEXT("Creating odometry...");

    auto parallel_tracker = std::make_shared<pros::v5::Rotation>(config::PORT_PARALLEL_TRACKER);
    parallel_tracker->set_data_rate(5);
    auto perpendicular_tracker = std::make_shared<pros::v5::Rotation>(config::PORT_PERPENDICULAR_TRACKER, true);
    perpendicular_tracker->set_data_rate(5);

    auto inertial = std::make_shared<pros::v5::IMU>(config::PORT_IMU);
    auto inertial_2 = std::make_shared<pros::v5::IMU>(config::PORT_IMU_2);

    auto odometry = std::make_shared<lib15442c::TrackerOdom>(
        parallel_tracker, perpendicular_tracker, inertial,
        config::INERTIAL_SCALE, config::TRACKER_WHEEL_DIAMETER * lib15442c::PI,
        config::PERPENDICULAR_TRACKER_OFFSET, config::PARALLEL_TRACKER_OFFSET,
        inertial_2 //, config::INERTIAL_SCALE_2
    );

    odometry->startTask();

    INFO_TEXT("Successfully created odometry!");

    return odometry;
}

std::shared_ptr<lib15442c::IDrivetrain> create_drivetrain()
{
    INFO_TEXT("Creating drivetrain...");

    auto left_motors = std::make_shared<pros::v5::MotorGroup>(config::PORT_LEFT_DRIVETRAIN, pros::v5::MotorGears::blue);
    auto right_motors = std::make_shared<pros::v5::MotorGroup>(config::PORT_RIGHT_DRIVETRAIN, pros::v5::MotorGears::blue);

    auto drivetrain = std::make_shared<lib15442c::TankDrive>(
        left_motors, right_motors,
        config::DRIVE_WHEEL_DIAMETER,
        config::DRIVE_GEAR_RATIO,
        config::TRACK_WIDTH);

    INFO_TEXT("Successfully created drivetrain!");

    return drivetrain;
}
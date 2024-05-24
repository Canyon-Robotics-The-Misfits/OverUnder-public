#pragma once

#include "main.h"
#include "lib15442c/chasis/drive_controller.hpp"

#define ArcDirection lib15442c::DriveController::ArcDirection

#define AUTO_PARAMETERS std::shared_ptr<lib15442c::IOdometry> odometry, std::shared_ptr<lib15442c::IDrivetrain> drivetrain, std::shared_ptr<lib15442c::DriveController> drive_controller, std::shared_ptr<pros::MotorGroup> catapult, std::shared_ptr<pros::MotorGroup> intake, std::shared_ptr<lib15442c::Pneumatic> frontLeftWing, std::shared_ptr<lib15442c::Pneumatic> frontRightWing, std::shared_ptr<lib15442c::Pneumatic> climb_pto, std::shared_ptr<lib15442c::Pneumatic> climb_up
#define AUTO_INPUTS odometry, drivetrain, drive_controller, catapult, intake, frontLeftWing, frontRightWing, climb_pto, climb_up

void closeAWP(AUTO_PARAMETERS);
void closeElims(AUTO_PARAMETERS);
void closeSide(AUTO_PARAMETERS);
void closeStart(AUTO_PARAMETERS, bool skipChannelPush = false);
void farAWP(AUTO_PARAMETERS);
void farElims(AUTO_PARAMETERS);
void farSide(AUTO_PARAMETERS);
void farStart(AUTO_PARAMETERS);

void skills(AUTO_PARAMETERS);
void startSkills(AUTO_PARAMETERS);

#define BIG_NUMBER 999999
#define DRIVE_TIME(speed, time) drivetrain->move(speed, 0); pros::delay(time); drivetrain->move(0, 0);
#define DRIVE_TIME_ANGLE(speed, time, angle) drive_controller->setMinSpeed(fabs(speed)); drive_controller->setMaxSpeed(fabs(speed)); drive_controller->setTimeout(time); drive_controller->drive(BIG_NUMBER * lib15442c::sgn(speed), angle);
#define TANK_TIME (left, right, time) drivetrain->tank(left, right); pros::delay(time); drivetrain->tank(0, 0);
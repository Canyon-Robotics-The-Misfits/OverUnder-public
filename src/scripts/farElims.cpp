#include "auto.hpp"
#include "main.h"
#include <iostream>

void farElims(AUTO_PARAMETERS)
{
    farStart(AUTO_INPUTS);

    // Go near the elevation bar
    frontLeftWing->retract();
    drivetrain->tank(0, -127);
    // waitUntil(odometry->getRotation() <= 180);
    pros::delay(400);
    drivetrain->tank(0, 0);

    drive_controller->setThreshold(8); // Speed > Accuracy
    drive_controller->faceAngle(183);
    drivetrain->tank(127, 127); // Go around barrier
    pros::delay(225);
    drivetrain->set_brake_mode(pros::E_MOTOR_BRAKE_COAST); // Coast into barrier
    drive_controller->setThreshold(36);
    drive_controller->setMinSpeed(100);
    drive_controller->boomerang(pos(144 - 37, 72 + 19.5), INFINITY, false, 0.6, 3, 15, 0.8);

    drive_controller->faceAngle(270);
}
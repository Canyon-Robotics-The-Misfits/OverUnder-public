#include "auto.hpp"
#include "main.h"
#include <iostream>

void closeSide(AUTO_PARAMETERS)
{
    closeStart(AUTO_INPUTS, true);

    // Touch the side of the elevation bar
    drive_controller->setThreshold(3.5);
    drive_controller->boomerang(pos(122, 28), 30, true, 0.5, 3, -1);
    drive_controller->setChained(true);
    drive_controller->setMinSpeed(127);
    drive_controller->drive(-6);
    drive_controller->setThreshold(2);
    drive_controller->boomerang(pos(107, 67), 182.5, true, 0.8);
    DRIVE_TIME(-60, 200);
    frontRightWing->extend();
    pros::delay(100);
    drivetrain->tank(-40, 40);
    pros::delay(500);
    drivetrain->tank(0, 0);
}
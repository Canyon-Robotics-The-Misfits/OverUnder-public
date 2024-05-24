#include "auto.hpp"
#include "main.h"
#include <iostream>
#include "config.hpp"

void farAWP(AUTO_PARAMETERS)
{
    farStart(AUTO_INPUTS);

    // Create the distance sensor
    std::shared_ptr<pros::Distance> distance = std::make_shared<pros::Distance>(config::PORT_DISTANCE);

    // Back up from goal and turn around
    frontLeftWing->retract();
    DRIVE_TIME(-127, 100);
    drivetrain->tank(-40, -127);
    pros::delay(250);
    drivetrain->tank(0, 0);

    // Go next to elevation bar
    intake->move(-127); // Outake in case triball is in intake and will be detected by distance sensor
    drive_controller->setChained(true);
    drive_controller->setThreshold(8);
    drive_controller->faceAngle(135);
    drive_controller->setChained(true);
    drive_controller->setThreshold(41);
    drive_controller->setTimeout(2500);
    drive_controller->setMaxSpeed(105);
    drive_controller->setMinSpeed(70);
    drive_controller->boomerang(pos(144 - 12.25, 72 + 7.5), 180, false, 0.55, 50, 10, 0.8);
    
    // Drive until distance sensor detects elevation bar
    // drive_controller->setThreshold(5);
    // drive_controller->faceAngle(180);
    drivetrain->tank(35, 35);
    waitUntil(distance->get() <= 350);
    drivetrain->tank(0, 0);
}
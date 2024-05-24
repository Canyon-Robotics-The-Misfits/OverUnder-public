#include "auto.hpp"
#include "main.h"
#include <iostream>
#include "config.hpp"

#define LOGGER "Close Start"

void closeStart(AUTO_PARAMETERS, bool skipChannelPush)
{
    // Create the distance sensor
    std::shared_ptr<pros::Distance> distance = std::make_shared<pros::Distance>(config::PORT_DISTANCE);

    int startingTime = pros::millis();
    odometry->setRotation(270);
    odometry->setPosition(pos(144 - 15.75, 38));

    // Pick up the first triball
    intake->move(127);
    drive_controller->setThreshold(16);
    drive_controller->setMaxSpeed(127);
    drive_controller->boomerang(pos(72 + 2.25, 48 - 1.5));

    // Push the neutral zone triball to the other side of the field
    drivetrain->tank(-127, -127);
    pros::delay(50);
    drivetrain->tank(0, -127);
    pros::delay(200);
    drive_controller->setThreshold(5);
    drive_controller->faceAngle(0);
    intake->move(0);
    frontLeftWing->extend();
    DRIVE_TIME_ANGLE(127, 525, 0);
    frontLeftWing->retract();
    drivetrain->tank(-127, 0);
    pros::delay(150);
    drive_controller->setThreshold(5);
    drive_controller->faceAngle(300);
    intake->move(0);

    // Go to match load zone
    drivetrain->tank(-70, -115);
    pros::delay(500);
    drive_controller->setMaxSpeed(70);
    drive_controller->boomerang(pos(109, 13), 0, true, 0.6, 3, -1, 0.8);

    // Remove the triball from the match load zone
    drive_controller->setTimeout(500);
    drive_controller->setThreshold(5);
    drive_controller->faceAngle(55);
    frontRightWing->extend();
    frontLeftWing->extend();
    drive_controller->setTimeout(750);
    drive_controller->drive(14.5, 55);
    drive_controller->setThreshold(15);
    drive_controller->setMinSpeed(127);
    drive_controller->turn(-60);
    drive_controller->faceAngle(35);
    frontRightWing->retract();

    // Wait until the end to push the triballs across the channel
    waitUntil(pros::millis() - startingTime >= 11750);

    // Push the triballs across the channel
    frontLeftWing->retract();
    DRIVE_TIME(70, 80); // Go around barrier
    drive_controller->setThreshold(18);
    drive_controller->setTimeout(3000);
    drive_controller->setMaxSpeed(50);
    drive_controller->boomerang(pos(144 - 19.5, 72 - 3), 0, false, 0.4, 7, -1, 0.8);
    drive_controller->faceAngle(0);
    intake->move(-127);
    drivetrain->tank(35, 35);
    pros::delay(500); // Outake the triball before looking at distance sensor so it doesn't sense the triball
    waitUntil(distance->get() <= 350);
    
}
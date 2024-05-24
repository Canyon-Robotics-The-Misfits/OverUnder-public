#include "auto.hpp"
#include "config.hpp"
#include "main.h"
#include <iostream>

void startSkills(AUTO_PARAMETERS)
{
    odometry->setRotation(245);
    odometry->setPosition(pos(144 - 11, 26));

    // Push the triballs into the goal
    drivetrain->tank(127, 100);
    intake->move(127); // Drop the intake
    pros::delay(100);
    intake->move(-127);
    pros::delay(700);

    // Drive to the match load zone
    drive_controller->setTimeout(750);
    drive_controller->setThreshold(0.5); // Accuracy is important here
    drive_controller->boomerang(pos(144 - 27, 10), INFINITY, true);
    intake->move(0);

    // Go to the correct angle to touch the match load bar
    lib15442c::Point goal = pos(74, 120);
    // drive_controller->setThreshold(2);
    // drive_controller->facePoint(goal);

    // Drive to the match load bar and start shooting
    // float angleToGoal = angle_between(odometry->getPosition(), goal);
    // DRIVE_TIME_ANGLE(-80, 100, angleToGoal);

    drive_controller->setThreshold(2);
    drive_controller->facePoint(goal);
    // pros::delay(250); // Wait for possible drift (don't know if this helps)

    // New
    float angleToGoal = angle_between(odometry->getPosition(), goal);
    DRIVE_TIME_ANGLE(-70, 50, angleToGoal);
    drive_controller->facePoint(goal);

    float startingRotation = odometry->getRotation();
    float startingRotation2 = odometry->getRotation2();
    float startingX = odometry->getX();
    float startingY = odometry->getY();
    catapult->move_velocity(config::CATA_MATCH_LOAD_SPEED);
    // catapult->move(127);
    pros::delay(25000);
    // pros::delay(1000);
    catapult->move(0);
    pros::delay(150); // Wait for inertial drift to stop
    // int rotation_offset = odometry->getRotation2() - startingRotation2;
    // odometry->setRotation(startingRotation + rotation_offset);
    odometry->setRotation(startingRotation);
    odometry->setPosition(pos(startingX, startingY));

    // Push the triballs along the barrier
    DRIVE_TIME(100, 200);

    intake->move(-127);
    drive_controller->setTimeout(2500);
    // drive_controller->setThreshold(24);
    drive_controller->setThreshold(19);
    drive_controller->setMinSpeed(110);
    drive_controller->boomerang(pos(120 - 26.5, 48 - 6), 340, false, 0.6);
    
    frontLeftWing->extend();
    
    drive_controller->setThreshold(5);
    drive_controller->faceAngle(270);
    frontRightWing->extend();

    // DRIVE_TIME_ANGLE(127, 1300, 270);
    drive_controller->setAsync(true);
    drive_controller->setTimeout(2500);
    drive_controller->drive(BIG_NUMBER, 270);
    waitUntil(odometry->getX() <= 39.5);
    drive_controller->stopAllTasks();
    frontLeftWing->retract();

    DRIVE_TIME(-127, 200);
    drivetrain->tank(-127, 127);
    pros::delay(300);
    frontRightWing->retract();
    intake->move(0);
}
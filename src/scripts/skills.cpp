#include "auto.hpp"
#include "config.hpp"
#include "main.h"
#include <iostream>

void skills(AUTO_PARAMETERS)
{
    // Start of skills
    startSkills(AUTO_INPUTS);

    // Drive to the other side of the field
    intake->move(127);
    drive_controller->setTimeout(2000);
    drive_controller->boomerang(pos(29, 13), INFINITY, false, 0.6, 3, 15, 0.8);
    drive_controller->setThreshold(3);
    intake->move(0);
    drive_controller->faceAngle(300);
    
    // Drive through the alley and push in the goal
    // frontRightWing->extend();
    intake->move(-127);
    DRIVE_TIME(127, 250);
    intake->move(0);
    drive_controller->setAsync(true);
    drive_controller->setChained(true); // Technically doesn't chain but we don't want it to stop
    drive_controller->setMinSpeed(127);
    drive_controller->setMaxSpeed(127);
    drive_controller->setTimeout(3000);
    drive_controller->drive(BIG_NUMBER, 0.75);
    waitUntil(odometry->getY() >= 88);
    drive_controller->stopAllTasks();
    frontRightWing->extend();
    drivetrain->tank(127, 25);
    pros::delay(350);
    DRIVE_TIME(127, 300);
    intake->move(-127);
    DRIVE_TIME_ANGLE(127, 700, 80);

    // Push in again
    drivetrain->tank(-127, -90);
    pros::delay(350);
    intake->move(0);
    DRIVE_TIME(127, 250);
    DRIVE_TIME_ANGLE(127, 500, 85);
    drivetrain->tank(-127, -25);
    pros::delay(175);
    frontRightWing->retract();

    // First front push
    drive_controller->setThreshold(5);
    drive_controller->faceAngle(170);
    drive_controller->setMaxSpeed(110);
    drive_controller->boomerang(pos(37, 88), 180, false, 0.6, 10, 15, 5);
    drivetrain->tank(10, 127);
    drive_controller->awaitAngle(34, 25, false);
    drive_controller->setThreshold(5);
    drive_controller->facePoint(pos(72 - 15, 120));
    DRIVE_TIME(127, 700);
    pros::delay(100);

    // Second front push
    drive_controller->setTimeout(500);
    drive_controller->faceAngle(20);
    DRIVE_TIME(-127, 150);
    drive_controller->setMaxSpeed(80);
    drive_controller->setThreshold(5);
    drive_controller->boomerang(pos(72 - 7, 72 + 10), 270, true, 0.2, 9, -1, 0.8);
    drive_controller->setThreshold(5);
    drive_controller->faceAngle(0);
    frontLeftWing->extend();
    DRIVE_TIME_ANGLE(90, 1150, 0);
    frontLeftWing->retract();
    intake->move(-127);
    DRIVE_TIME(-127, 200);

    // Third front push
    drive_controller->boomerang(pos(86.5, 72 + 8.5), 285, true, 0.35, 3, 15, 1);
    intake->move(0);
    drive_controller->setThreshold(5);
    drive_controller->faceAngle(350);
    frontLeftWing->extend();
    drive_controller->setMaxSpeed(70);
    drive_controller->setMinSpeed(70);
    drive_controller->setTimeout(800);
    drive_controller->setChained(true);
    drive_controller->drive(BIG_NUMBER);
    frontLeftWing->retract();
    drivetrain->tank(-60, -127);
    pros::delay(250);
    DRIVE_TIME_ANGLE(-127, 550, 45);

    // Go to side of goal
    // drive_controller->setThreshold(10);
    // drive_controller->faceAngle(90);
    // drive_controller->setChained(true);
    // drive_controller->setMinSpeed(80);
    // drive_controller->boomerang(pos(87, 86.5), 90, false, 0.8);
    // frontRightWing->extend();
    // drivetrain->tank(0, 127);
    // drive_controller->awaitAngle(45, 25, false);
    // frontRightWing->retract();
    // drive_controller->setTimeout(2000);
    // drive_controller->boomerang(pos(111, 112.5));

    // Go to side of goal
    drive_controller->faceAngle(45);
    drive_controller->setTimeout(2000);
    drive_controller->boomerang(pos(111, 112.5));
    
    // Push in the side of the goal
    drive_controller->setThreshold(5);
    drive_controller->faceAngle(315);
    drivetrain->tank(80, 127);
    pros::delay(800);
    drivetrain->tank(-50, -127);
    pros::delay(400);

    // Push in again
    drivetrain->tank(80, 127);
    pros::delay(900);
    drivetrain->tank(-50, -127);
    pros::delay(500);

    // Elevate C-Tier
    intake->move(-127);
    drive_controller->setThreshold(5);
    drive_controller->faceAngle(135);
    intake->move(0);
    DRIVE_TIME_ANGLE(127, 350, 135);
    climb_up->extend();
    drive_controller->faceAngle(180);
    DRIVE_TIME_ANGLE(110, 1000, 180);
    climb_pto->extend();
    pros::delay(50); // Wait for pto to activate
    DRIVE_TIME(127, 500);

    // // Final goal push in
    // // drive_controller->setThreshold(5);
    // // drive_controller->faceAngle(235);
    // frontLeftWing->retract();
    // frontRightWing->retract();
    // climb_up->extend();
    // drive_controller->setChained(true);
    // drive_controller->setMinSpeed(60);
    // drive_controller->drive(30);
    // drive_controller->setThreshold(2);
    // drive_controller->boomerang(pos(93, 86), INFINITY, true);
    // drive_controller->setThreshold(5);
    // drive_controller->faceAngle(0);
    // frontLeftWing->extend();
    // frontRightWing->extend();
    // DRIVE_TIME(127, 550);

    // // Elevate
    // frontLeftWing->retract();
    // frontRightWing->retract();
    // DRIVE_TIME(-127, 150);
    // drive_controller->facePoint(pos(120, 72));
    // DRIVE_TIME(-80, 400);
    // climb_pto->retract();
    // DRIVE_TIME(127, 3000);
    
}
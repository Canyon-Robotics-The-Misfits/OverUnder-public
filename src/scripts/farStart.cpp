#include "auto.hpp"
#include "main.h"
#include <iostream>

void farStart(AUTO_PARAMETERS)
{
    // Set the starting position
    odometry->setRotation(270);
    odometry->setPosition(pos(144 - 17, 144 - 37.75));

    // Push the preload to the side of the goal
    frontRightWing->extend();
    
    // Grab the triball in the neutral zone next to the barrier
    DRIVE_TIME(127, 175);
    frontRightWing->retract();
    intake->move(127);
    drive_controller->setThreshold(25.75);
    drive_controller->boomerang(pos(72 - 6, 72 + 5), 235, false, 0.4, 3, -1);

    // Push both nuetral zone triballs into the goal
    drive_controller->setAsync(true);
    drive_controller->setChained(true); // NEW
    drive_controller->setMaxSpeed(90); // Slow down turn so it doesn't mess up other triball as much if it hits
    drive_controller->setThreshold(8);
    drive_controller->faceAngle(10);
    pros::delay(300);
    intake->move(0);
    drive_controller->awaitDone();
    frontLeftWing->extend();
    frontRightWing->extend();
    intake->move(-127);
    drive_controller->setTimeout(700);
    drive_controller->setMinSpeed(127);
    drive_controller->drive(50, 0);
    DRIVE_TIME(-127, 350); 
    frontLeftWing->retract();
    frontRightWing->retract();
    intake->move(0);

    // Rotate until the robot is facing the third triball and pick it up
    // drivetrain->tank(-90, 0);
    // pros::delay(100);
    // drivetrain->tank(-110, 30);
    // pros::delay(150);
    // drivetrain->tank(-110, 20);
    lib15442c::Point thirdTriball = pos(100, 75);
    drive_controller->setChained(true);
    drive_controller->setTimeout(750);
    drive_controller->setThreshold(5);
    drive_controller->facePoint(thirdTriball, -5);
    // int turnToTriballStartTime = pros::millis();
    // waitUntil(odometry->getRotation() < angle_between(odometry->getPosition(), thirdTriball) + 180 + 38 || pros::millis() - turnToTriballStartTime > 2000);
    intake->move(127);
    drive_controller->setMaxSpeed(100);

    drive_controller->setThreshold(25);
    drive_controller->boomerang(thirdTriball, INFINITY, false, 0.6, 3, 15, 0.8);
    pros::delay(100);

    // Push the third triball next to the side of the goal
    drive_controller->setTimeout(500);
    drive_controller->faceAngle(200); // Accuracy is needed here
    drive_controller->setChained(true);
    drive_controller->setMinSpeed(127);
    drive_controller->setThreshold(1);
    drive_controller->drive(-6.5);
    intake->move(0);

    // drive_controller->setAsync(true);
    drive_controller->setThreshold(2);
    drive_controller->faceAngle(322, -11.25); // Larger angles make the robot drive backwards in a straight line after the turn
    int turnToAngleStartTime = pros::millis();
    // waitUntil(odometry->getRotation() >= 275 || pros::millis() - turnToAngleStartTime > 2000);
    intake->move(-127);
    frontRightWing->extend();
    // drive_controller->awaitDone();

    // Remove the triball from the match load zone
    DRIVE_TIME_ANGLE(60, 250, 315);
    drivetrain->tank(-127, 127);
    pros::delay(250);
    frontLeftWing->extend();
    drive_controller->setChained(true); // NEW
    drive_controller->setThreshold(7);
    drive_controller->faceAngle(325);
    frontRightWing->retract();

    // Push the third triball, preload, and match load into the goal
    intake->move(-127);
    drivetrain->tank(30, 127);
    pros::delay(450);
    drivetrain->tank(127, 127);
    pros::delay(150);
    drivetrain->tank(0, 0);
    pros::delay(100);

    // Back up from the goal
    frontLeftWing->retract();
    DRIVE_TIME(-127, 100);
    drivetrain->tank(-40, -127);
    // waitUntil(odometry->getRotation() <= 180);
    pros::delay(350);
    drivetrain->tank(0, 0);

    // Pick up the triball under the elevation bar
    intake->move(127);
    drive_controller->setThreshold(15);
    drive_controller->setTimeout(2500);
    drive_controller->setMaxSpeed(95); // Accuracy is needed over speed
    drive_controller->boomerang(pos(144 - 12.25, 72 + 7.5), 180, false, 0.55, 22, 10, 0.8);

    // Score the triball in the goal
    drive_controller->setAsync(true);
    drive_controller->setTimeout(2000);
    drive_controller->setThreshold(2);
    drive_controller->boomerang(pos(144 - 17.5, 120), 358, true);
    pros::delay(400);
    intake->move(0);
    drive_controller->awaitDone();
    drive_controller->setChained(true);
    drive_controller->setThreshold(10);
    drive_controller->faceAngle(310);
    intake->move(-127);
    frontLeftWing->extend();
    drivetrain->tank(127, 127);
    pros::delay(250);
    drivetrain->tank(60, 127);
    pros::delay(700);
    intake->move(0);
}
#include "auto.hpp"
#include "config.hpp"
#include "lib15442c/screen/screen.hpp"
#include "main.h"
#include "pros/adi.hpp"
#include "pros/misc.h"
#include "pros/motor_group.hpp"
#include "pros/rtos.hpp"

#define LOGGER "Opcontrol"

constexpr float a = 70;
constexpr float b = 147;
float curve_joystick(float value)
{
    if (value != 0)
    {
        float toT = fabs(value) / 127.0;
        float output = a * pow(1 - toT, 2) * toT + b * (1 - toT) * pow(toT, 2) + 127 * pow(toT, 3);
        return output * lib15442c::sgn(value);
    }
    else
    {
        return 0;
    }
}

void opcontrol()
{
    INFO_TEXT("Driver Control started");

    // Initialize all the devices
    pros::Controller master(CONTROLLER_MASTER);
    const char *ControllerRumblePattern = "";

    std::shared_ptr<lib15442c::TrackerOdom> odometry = create_tracker_odometry();
    std::shared_ptr<lib15442c::IDrivetrain> drivetrain = create_drivetrain();
    std::shared_ptr<pros::MotorGroup> catapult = std::make_shared<pros::MotorGroup>(config::PORT_CATA_MOTORS, pros::v5::MotorGears::red);
    int macthLoadSpeedOffset = 0;
    int machLoadSpeedIncrement = 5;
    std::shared_ptr<pros::MotorGroup> intake = std::make_shared<pros::MotorGroup>(config::PORT_INTAKE_MOTORS);

    std::shared_ptr<lib15442c::Pneumatic> frontLeftWing = std::make_shared<lib15442c::Pneumatic>(config::PORT_FRONT_LEFT_WING, false);
    std::shared_ptr<lib15442c::Pneumatic> frontRightWing = std::make_shared<lib15442c::Pneumatic>(config::PORT_FRONT_RIGHT_WING, false);

    std::shared_ptr<lib15442c::Pneumatic> climb_up = std::make_shared<lib15442c::Pneumatic>(config::PORT_CLIMB_UP, false);

    std::shared_ptr<lib15442c::Pneumatic> climb_pto = std::make_shared<lib15442c::Pneumatic>(config::PORT_CLIMB_PTO, false);

    intake->set_brake_mode(pros::E_MOTOR_BRAKE_COAST);
    catapult->set_brake_mode(pros::E_MOTOR_BRAKE_COAST);
    odometry->setPosition(pos(0, 0));


    bool readyForElevation = false;
    bool matchLoading = false;
    bool isCoast = true;
    bool drivetrainReversed = false;

    // Run startSkills if skills is selected
    lib15442c::Button selectedButton = screen.getSelectedButton();
    lib15442c::ButtonIds buttonId = selectedButton.id;
    if (buttonId == lib15442c::ButtonIds::Skills)
    {
        auto drive_pid = std::make_shared<lib15442c::PID>(config::DRIVE_P, config::DRIVE_D, 0, 0, config::DRIVE_I);
        auto turn_pid = std::make_shared<lib15442c::PID>(config::TURN_P, config::TURN_D, 0, 0, config::TURN_I);

        std::shared_ptr<lib15442c::DriveController> drive_controller = std::make_shared<lib15442c::DriveController>(drivetrain, odometry, drive_pid, turn_pid);

        drivetrain->set_brake_mode(pros::E_MOTOR_BRAKE_BRAKE);
        bool taskDone = false;
        pros::Mutex taskDoneMutex;
        pros::Task startSkillsTask([AUTO_INPUTS, &taskDone, &taskDoneMutex] { 
            startSkills(AUTO_INPUTS); 
            taskDoneMutex.lock();
            taskDone = true;
            taskDoneMutex.unlock();
        });
        while (true)
        {
            // Stop the start skills task if any thumb button is pressed
            if (master.get_digital_new_press(DIGITAL_UP) || master.get_digital_new_press(DIGITAL_DOWN) || master.get_digital_new_press(DIGITAL_LEFT) || master.get_digital_new_press(DIGITAL_RIGHT) || master.get_digital_new_press(DIGITAL_A) || master.get_digital_new_press(DIGITAL_B) || master.get_digital_new_press(DIGITAL_Y) || master.get_digital_new_press(DIGITAL_X))
            {
                startSkillsTask.remove();
                startSkillsTask.join();
                drivetrain->move(0, 0);
                break;
            }
            // If the start skills task is done, break the loop
            
            taskDoneMutex.lock();
            bool taskDoneValue = taskDone;
            taskDoneMutex.unlock();

            if (taskDoneValue)
            {
                drivetrain->move(0, 0);
                break;
            }

            pros::delay(40);
        }
    }

    drivetrain->set_brake_mode(pros::E_MOTOR_BRAKE_COAST);

    int ControllerTimer = 0;
    while (true)
    {
        // Toggle drivetrain reverse and brake/coast
        if (master.get_digital_new_press(DIGITAL_DOWN))
        {
            if (master.get_digital(DIGITAL_LEFT))
            {
                // Toggle the drivetrain between brake and coast
                isCoast = !isCoast;
                drivetrain->set_brake_mode(isCoast ? pros::E_MOTOR_BRAKE_COAST : pros::E_MOTOR_BRAKE_BRAKE);
            }
            else
            {
                // Reverse the drivetrain
                drivetrainReversed = !drivetrainReversed;
            }
            ControllerRumblePattern = ".";
        }

        float linear_speed = curve_joystick(master.get_analog(ANALOG_LEFT_Y));
        float rotational_speed = master.get_analog(ANALOG_LEFT_X);

        if (drivetrainReversed)
        {
            linear_speed *= -1;
        }
        
        if (sqrt(linear_speed * linear_speed + rotational_speed * rotational_speed) < 12) {
            linear_speed = 0;
            rotational_speed = 0;
        }

        drivetrain->move(linear_speed, rotational_speed);

        // Catapult controls
        if (master.get_digital_new_press(DIGITAL_B)) {
            matchLoading = !matchLoading;
        }

        if (matchLoading) {
            catapult->move_velocity(config::CATA_MATCH_LOAD_SPEED + macthLoadSpeedOffset);
            // catapult->move(127);
        } else {
            catapult->move_velocity(0);
        }

        // Intake controls
        if (master.get_digital(DIGITAL_R1))
        {
            intake->move(127);
        }
        else if (master.get_digital(DIGITAL_L1))
        {
            intake->move(-127);
        }
        else
        {
            intake->brake();
        }

        // Wing controls
        if (master.get_digital_new_press(DIGITAL_L2))
        {
            if (!drivetrainReversed)
            {
                frontLeftWing->toggle();
            }
            else
            {
                frontRightWing->toggle();
            }
        }
        if (master.get_digital_new_press(DIGITAL_R2))
        {
            if (!drivetrainReversed)
            {
                frontRightWing->toggle();
            }
            else
            {
                frontLeftWing->toggle();
            }
        }
        if (master.get_digital_new_press(DIGITAL_A))
        {
            if (master.get_digital(DIGITAL_LEFT))
            {
                macthLoadSpeedOffset += machLoadSpeedIncrement;
                // Don't let the speed go above 200
                if ((config::CATA_MATCH_LOAD_SPEED + macthLoadSpeedOffset) * 2 > 200)
                {
                    macthLoadSpeedOffset -= machLoadSpeedIncrement;
                    ControllerRumblePattern = "..";
                }
                else
                {
                    ControllerRumblePattern = "-";
                }
            }
            else
            {
                // If one wing is open and the other is closed, close both
                if (frontLeftWing->get_value() != frontLeftWing->get_value())
                {
                    frontLeftWing->retract();
                    frontRightWing->retract();
                }
                // If both wings are closed, open both
                else if (!frontLeftWing->get_value())
                {
                    frontLeftWing->extend();
                    frontRightWing->extend();
                }
                // If both wings are open, close both
                else
                {
                    frontLeftWing->retract();
                    frontRightWing->retract();
                }
            }
        }

        // Catapult speed controls
        if (master.get_digital_new_press(DIGITAL_Y))
        {
            if (master.get_digital(DIGITAL_LEFT))
            {
                macthLoadSpeedOffset -= machLoadSpeedIncrement;
                // Don't let the speed go below 0
                if ((config::CATA_MATCH_LOAD_SPEED + macthLoadSpeedOffset) * 2 < 0)
                {
                    macthLoadSpeedOffset += machLoadSpeedIncrement;
                    ControllerRumblePattern = "..";
                }
                else
                {
                    ControllerRumblePattern = "-";
                }
            }
        }

        // Climb controls
        if (master.get_digital_new_press(DIGITAL_RIGHT))
        {
            climb_pto->toggle();
            ControllerRumblePattern = "..";
        }
        if (master.get_digital_new_press(DIGITAL_X))
        {
            climb_up->set_value(true);
        }

        // Climb macro
        if (master.get_digital_new_press(DIGITAL_UP))
        {
            if (!readyForElevation)
            {
                // Prepare for elevation
                readyForElevation = true;
                ControllerRumblePattern = "-";
                climb_pto->set_value(false);
                climb_up->set_value(true);
                frontLeftWing->retract();
                frontRightWing->retract();
                catapult->move(0);
            }
            else
            {
                // Elevate
                ControllerRumblePattern = "--";
                climb_pto->set_value(true);
                climb_up->set_value(false);
                frontLeftWing->extend();
                frontRightWing->extend();
            }
        }

        // PID tuning controls
        // if(master.get_digital_new_press(DIGITAL_UP)){
        //     p += 0.5;
        //     drive_pid->setP(p);
        // }
        // if(master.get_digital_new_press(DIGITAL_DOWN)){
        //     p -= 0.5;
        //     drive_pid->setP(p);
        // }

        // if(master.get_digital_new_press(DIGITAL_X)){
        //     d += 0.5;
        //     drive_pid->setD(d);
        // }
        // if(master.get_digital_new_press(DIGITAL_B)){
        //     d -= 0.5;
        //     drive_pid->setD(d);
        // }

        // if(master.get_digital_new_press(DIGITAL_A)){
        //     // drive_controller->turn(90);
        //     // drive_controller->turn(90);
        //     // drive_controller->turn(90);
        //     // drive_controller->turn(90);

        //     drive_controller->drive(48);
        //     drive_controller->drive(-48);
        //     drive_controller->drive(24);
        //     drive_controller->drive(-12);
        //     drive_controller->drive(36);
        //     drive_controller->drive(-6);
        //     drive_controller->drive(-12);
        //     drive_controller->drive(2);
        //     drive_controller->turn(90);
        // }

        if (ControllerTimer % 1000 == 0)
        {
            // std::cout << odometry->getX() << ", " << odometry->getY() << ", " << odometry->getRotation() << std::endl;
            if (strlen(ControllerRumblePattern) > 0)
            {
                // master.rumble(ControllerRumblePattern);
                ControllerRumblePattern = "";
            }
            else
            {
                std::vector<const char *> temp_levels = drivetrain->get_temp_levels();
                std::string motorTemps = "";
                for (int i = 0; i < int(temp_levels.size()); i++)
                {
                    motorTemps += temp_levels[i];
                    if (i == 2)
                        motorTemps += " ";
                }

                std::string output = "";

                if (master.get_digital(DIGITAL_LEFT))
                {
                    std::string selectedButtonText = screen.getSelectedButton().text;
                    if (selectedButtonText.length() == 0)
                    {
                        selectedButtonText = "No Auto Selected";
                    }
                    output += selectedButtonText + "                     ";
                }
                else
                {
                    output += drivetrainReversed ? "R" : "F";
                    output += " ";
                    output += isCoast ? "C" : "B";
                    output += " ";
                    output += motorTemps;
                    output += " ";
                    output += std::to_string(int(config::CATA_MATCH_LOAD_SPEED + macthLoadSpeedOffset) * 2);
                }

                // master.set_text(2, 0, output + "                     "); // Spaces clear the line

                // // TODO: Delete me
                // p = round(p * 100) / 100;
                // d = round(d * 100) / 100;
                // std::string strP = std::to_string (p);
                // strP.erase ( strP.find_last_not_of('0') + 1, std::string::npos );
                // strP.erase ( strP.find_last_not_of('.') + 1, std::string::npos );
                // std::string strD = std::to_string (d);
                // strD.erase ( strD.find_last_not_of('0') + 1, std::string::npos );
                // strD.erase ( strD.find_last_not_of('.') + 1, std::string::npos );
                // master.set_text(2, 0, "P: " + strP + "  D: " + strD + "    ");
            }
        }

        ControllerTimer += 20;
        pros::delay(20);
    }
}
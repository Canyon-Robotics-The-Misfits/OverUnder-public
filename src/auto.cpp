#include "main.h"
#include "auto.hpp"
#include "config.hpp"
#include "lib15442c/chasis/tank_drive.hpp"

#define LOGGER "Auto"

using namespace lib15442c;

void autonomous()
{
    INFO_TEXT("Autonomous started");
    int startingTime = pros::millis();

    std::shared_ptr<lib15442c::TrackerOdom> odometry = create_tracker_odometry();
    std::shared_ptr<lib15442c::IDrivetrain> drivetrain = create_drivetrain();
    std::shared_ptr<pros::MotorGroup> catapult = std::make_shared<pros::MotorGroup>(config::PORT_CATA_MOTORS, pros::v5::MotorGears::red);

    std::shared_ptr<pros::MotorGroup> intake = std::make_shared<pros::MotorGroup>(config::PORT_INTAKE_MOTORS);
    intake->set_brake_mode(pros::E_MOTOR_BRAKE_BRAKE);

    std::shared_ptr<lib15442c::Pneumatic> frontLeftWing = std::make_shared<lib15442c::Pneumatic>(config::PORT_FRONT_LEFT_WING, false);
    std::shared_ptr<lib15442c::Pneumatic> frontRightWing = std::make_shared<lib15442c::Pneumatic>(config::PORT_FRONT_RIGHT_WING, false);

    std::shared_ptr<lib15442c::Pneumatic> climb_pto = std::make_shared<lib15442c::Pneumatic>(config::PORT_CLIMB_PTO, false);
    std::shared_ptr<lib15442c::Pneumatic> climb_up = std::make_shared<lib15442c::Pneumatic>(config::PORT_CLIMB_UP, false);

    auto drive_pid = std::make_shared<lib15442c::PID>(config::DRIVE_P, config::DRIVE_D, 0, 0, config::DRIVE_I);
    auto turn_pid = std::make_shared<lib15442c::PID>(config::TURN_P, config::TURN_D, 0, 0, config::TURN_I);

    std::shared_ptr<lib15442c::DriveController> drive_controller = std::make_shared<lib15442c::DriveController>(drivetrain, odometry, drive_pid, turn_pid);

    Button selectedButton = screen.getSelectedButton();
    lib15442c::ButtonIds buttonId = selectedButton.id;


    drivetrain->set_brake_mode(pros::E_MOTOR_BRAKE_BRAKE);

    switch (buttonId) {
    case ButtonIds::CloseAWP:
        closeAWP(AUTO_INPUTS);
        break;
    case ButtonIds::CloseElims:
        closeElims(AUTO_INPUTS);
        break;
    case ButtonIds::CloseSide:
        closeSide(AUTO_INPUTS);
        break;
    case ButtonIds::FarAWP:
        farAWP(AUTO_INPUTS);
        break;
    case ButtonIds::FarElims:
        farElims(AUTO_INPUTS);
        break;
    case ButtonIds::FarSide:
        farSide(AUTO_INPUTS);
        break;
    case ButtonIds::Skills:
        skills(AUTO_INPUTS);
        break;
    default:
        break;
    }

    drive_controller->awaitDone();
    drivetrain->move(0, 0);

    int endingTime = pros::millis();
    INFO("Autonomous Ended; Time: %i ms", endingTime - startingTime);
}
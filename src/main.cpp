#include "main.h"
#include "config.hpp"
#include "lib15442c/logger.hpp"
#include "lib15442c/screen/screen.hpp"
#include "pros/imu.h"

#define LOGGER "Main"

void initialize()
{
    while (true)
    {
        float startTime = pros::millis();
        INFO_TEXT("Calibrating inertial...");
        pros::c::imu_reset(config::PORT_IMU);
        // pros::c::imu_reset(config::PORT_IMU_2);
        waitUntil(pros::c::imu_get_rotation(config::PORT_IMU) != PROS_ERR_F);
        // waitUntil(pros::c::imu_get_rotation(config::PORT_IMU_2) != PROS_ERR_F);

        // Don't calibrate again if calibration successful and took at least 1.5 seconds
        if (pros::millis() - startTime >= 1500)
        {
            INFO_TEXT("Successfully calibrated inertial!");
            break;
        }

        WARN_TEXT("Inertial calibration failed, retrying...");
    }

    INFO_TEXT("Initializing screen...");
    screen.addButtons({lib15442c::Button(20, 20, 133, 90, "Close AWP", pros::Color::red, pros::Color::green, lib15442c::ButtonIds::CloseAWP),
                       lib15442c::Button(20, 130, 133, 90, "Far AWP", pros::Color::blue, pros::Color::green, lib15442c::ButtonIds::FarAWP),
                       lib15442c::Button(173, 20, 133, 35, "Close Elims", pros::Color::red, pros::Color::green, lib15442c::ButtonIds::CloseElims),
                       lib15442c::Button(173, 75, 133, 35, "Close Side", pros::Color::red, pros::Color::green, lib15442c::ButtonIds::CloseSide),
                       lib15442c::Button(173, 130, 133, 35, "Far Elims", pros::Color::blue, pros::Color::green, lib15442c::ButtonIds::FarElims),
                       lib15442c::Button(173, 185, 133, 35, "Far Side", pros::Color::blue, pros::Color::green, lib15442c::ButtonIds::FarSide),
                       lib15442c::Button(326, 20, 133, 90, "Skills", pros::Color::purple, pros::Color::green, lib15442c::ButtonIds::Skills),
                       lib15442c::Button(326, 130, 133, 90, "Calibrate", pros::Color::orange, pros::Color::green, lib15442c::ButtonIds::Calibrate)});
    screen.registerScreenPressed();
    screen.startScreenThread();
    INFO_TEXT("Screen Initialized!");
}

void disabled()
{
    INFO_TEXT("Competition Disabled");
}

void competition_initialize()
{
    INFO_TEXT("Connected to field control");
}
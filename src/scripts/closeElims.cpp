#include "auto.hpp"
#include "main.h"
#include <iostream>

void closeElims(AUTO_PARAMETERS)
{
    closeStart(AUTO_INPUTS);

    // Leave the elevation bar
    pros::delay(150);
    drive_controller->setChained(true);
    drive_controller->setMinSpeed(80);
    drive_controller->drive(-36, 0);
}
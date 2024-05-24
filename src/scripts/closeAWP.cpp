#include "auto.hpp"
#include "main.h"
#include <iostream>

void closeAWP(AUTO_PARAMETERS)
{
    closeStart(AUTO_INPUTS);

    drivetrain->tank(-35, -35);
    pros::delay(200);
    drivetrain->tank(0, 0);
}
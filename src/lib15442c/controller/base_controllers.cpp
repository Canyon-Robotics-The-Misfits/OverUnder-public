#include <iostream>

#include "lib15442c/controller/base_controllers.hpp"

using namespace lib15442c;

float IErrorController::calculate(float current, float target)
{
    error = current - target;
    return calculateError(error);
}

float IErrorController::calculateError(float error)
{
    printf("Unimplemented!\n");
    return 0;
}

float IErrorController::getError()
{
    return error;
}

void IErrorController::reset()
{
    printf("Unimplemented!\n");
}
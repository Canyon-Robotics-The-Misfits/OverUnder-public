#include "lib15442c/controller/pid.hpp"
#include "lib15442c/util/math.hpp"
#include "iostream"

using namespace lib15442c;

PID::PID(float kP, float kD, float kI2, float kT, float kI) :
    kP(kP), kD(kD), kI2(kI2), kT(kT), kI(kI) {};

float PID::calculateError(float error)
{
    // Reset total error if the error passes 0
    if(sgn(error) != sgn(last_error)){
        total_error = 0;
    }

    // Add the error to the total error
    total_error += error;

    // Add the error and the time of that error to the partial errors
    partial_error_values.push_back(error);
    partial_error_times.push_back(pros::millis());

    // Remove all errors that are older than the time constant
    int current_time = pros::millis();
    while (partial_error_values.size() > 0 && partial_error_times[0] < current_time - kT) {
        partial_error_values.erase(partial_error_values.begin());
        partial_error_times.erase(partial_error_times.begin());
    }

    // Calculate the partial total error
    float partial_total_error = 0;
    for (size_t i = 0; i < partial_error_values.size(); i++) {
        partial_total_error += partial_error_values[i];
    }

    // std::cout << current_time << ", " << error * kP << ", " << error << std::endl;
    float output = (error * kP) + (total_error * kI) + (partial_total_error * kI2) + ((error - last_error) * kD);
    last_error = error;
    return output;
}

void PID::setP(float p)
{
    kP = p;
}

void PID::setD(float d)
{
    kD = d;
}

void PID::reset()
{
    last_error = 0;
    total_error = 0;
    partial_error_values = {};
    partial_error_times = {};
}
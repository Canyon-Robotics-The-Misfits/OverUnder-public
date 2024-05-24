#include "lib15442c/controller/base_controllers.hpp"
#include "lib15442c/logger.hpp"
#include "lib15442c/path/point.hpp"
#include "lib15442c/path/path.hpp"
#include "lib15442c/path/motion_profile.hpp"
#include "lib15442c/chasis/tank_drive.hpp"
#include "lib15442c/util/angles.hpp"
#include "lib15442c/util/math.hpp"
#include "pros/misc.h"
#include "pros/rtos.hpp"

#include <cmath>
#include <iostream>
#include <memory>
#include <numeric>

using namespace lib15442c;

lib15442c::TankDrive::TankDrive(
    std::shared_ptr<pros::v5::MotorGroup> left_motors,
    std::shared_ptr<pros::v5::MotorGroup> right_motors,
    float wheel_diameter,
    float gear_ratio,
    float track_width) : left_motors(left_motors),
                         right_motors(right_motors),
                         track_width(track_width),
                         deg_inch_ratio(wheel_diameter * PI * gear_ratio / 360)
{
    left_motors->set_brake_mode(pros::E_MOTOR_BRAKE_BRAKE);
    right_motors->set_brake_mode(pros::E_MOTOR_BRAKE_BRAKE);

    left_motors->tare_position();
    right_motors->tare_position();
}

void lib15442c::TankDrive::move(float linear_speed, float rot_speed)
{
    tank(linear_speed + rot_speed, linear_speed - rot_speed);
}

void lib15442c::TankDrive::move_ratio(float linear_speed, float rot_speed)
{
    if (fabs(linear_speed + rot_speed) > 127)
    {
        float total_speed = fabs(linear_speed + rot_speed);
        linear_speed = linear_speed / total_speed * 127;
        rot_speed = rot_speed / total_speed * 127;
    }
    else if (fabs(linear_speed - rot_speed) > 127)
    {
        float total_speed = fabs(linear_speed - rot_speed);
        linear_speed = linear_speed / total_speed * 127;
        rot_speed = rot_speed / total_speed * 127;
    }

    // if (rot_speed > linear_speed){
    //     rot_speed = linear_speed;
    // }
    // else if (rot_speed < -linear_speed){
    //     rot_speed = -linear_speed;
    // }

    // if(fabs(linear_speed + rot_speed) > 127){
    //     linear_speed = sgn(linear_speed) * 127 - rot_speed;
    // } else if (fabs(linear_speed - rot_speed) > 127){
    //     linear_speed = sgn(linear_speed) * 127 + rot_speed;
    // }

    move(linear_speed, rot_speed);
}

void lib15442c::TankDrive::move_speed(float linear_speed,
                                      float rotational_speed)
{
    pros::v5::MotorGears cartridge = left_motors->get_gearing(0);
    float start_rpm = 0;

    switch (cartridge)
    {
    case pros::v5::MotorGears::red:
    {
        start_rpm = 100;
    }
    break;
    case pros::v5::MotorGears::green:
    {
        start_rpm = 200;
    }
    break;
    case pros::v5::MotorGears::blue:
    {
        start_rpm = 600;
    }
    break;
    case pros::v5::MotorGears::invalid:
    {
        start_rpm = 600;
    }
    break;
    }

    float max_speed = start_rpm * 360 / 60 * deg_inch_ratio;

    move(linear_speed * (max_speed / 127.0),
         track_width * rad(rotational_speed) * (max_speed / 127.0));
}

void lib15442c::TankDrive::tank(float left, float right)
{
    left_motors->move(left);
    right_motors->move(right);
}


void lib15442c::TankDrive::tank_pid(float left_speed, float right_speed)
{
    // Convert from inches per second to RPM
    left_speed = left_speed / deg_inch_ratio / 360 * 60;
    right_speed = right_speed / deg_inch_ratio / 360 * 60;

    if (fabs(left_speed) < 5)
        left_speed = 0;
    if (fabs(right_speed) < 5)
        right_speed = 0;

    left_motors->move_velocity(left_speed);
    right_motors->move_velocity(right_speed);
}

void lib15442c::TankDrive::set_brake_mode(pros::motor_brake_mode_e_t mode)
{
    left_motors->set_brake_mode(mode);
    right_motors->set_brake_mode(mode);
}

float lib15442c::TankDrive::get_track_width()
{
    return track_width;
}

std::vector<const char*> lib15442c::TankDrive::get_temp_levels()
{
    std::vector<double> leftTemps = left_motors->get_temperature_all();
    std::vector<double> rightTemps = right_motors->get_temperature_all();
    std::vector<double> temps = leftTemps;
    // combine vectors
    temps.insert(temps.end(), rightTemps.begin(), rightTemps.end());

    std::vector<const char*> tempLevels;
    
    for(int i = 0; i < (int)temps.size(); i++){
        if(temps[i] >= 70){
            tempLevels.push_back("4");
        }
        else if(temps[i] >= 65){
            tempLevels.push_back("3");
        }
        else if(temps[i] >= 60){
            tempLevels.push_back("2");
        }
        else if (temps[i] >= 55){
            tempLevels.push_back("1");
        }
        else {
            tempLevels.push_back("0");
        }
    }

    return tempLevels;
}
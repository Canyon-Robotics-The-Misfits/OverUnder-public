#pragma once

#include <initializer_list>
#include <cstdint>

namespace config
{
    // PORTS
    constexpr std::initializer_list<std::int8_t> PORT_LEFT_DRIVETRAIN = {-16, -18, -17};
    constexpr std::initializer_list<std::int8_t> PORT_RIGHT_DRIVETRAIN = {12, 13, 14};

    constexpr int PORT_IMU = 21;
    constexpr int PORT_IMU_2 = 4; // TODO: get port
    // constexpr int PORT_GPS = 21;
    constexpr int PORT_PARALLEL_TRACKER = 3;
    constexpr int PORT_PERPENDICULAR_TRACKER = 7;
    constexpr int PORT_DISTANCE = 5;

    constexpr std::initializer_list<std::int8_t> PORT_CATA_MOTORS = {2};
    constexpr int PORT_CATA_ROT = 3;

    constexpr std::initializer_list<std::int8_t> PORT_INTAKE_MOTORS = {15};

    constexpr char PORT_FRONT_LEFT_WING = 'H';
    constexpr char PORT_FRONT_RIGHT_WING = 'G';

    constexpr char PORT_CLIMB_PTO = 'A';
    constexpr char PORT_CLIMB_UP = 'B';

    // SETTINGS
    constexpr float PARALLEL_TRACKER_OFFSET = -2.36853;
    constexpr float PERPENDICULAR_TRACKER_OFFSET = -6.03913;
    constexpr float TRACKER_WHEEL_DIAMETER = 2;

    constexpr float DRIVE_P = 19;
    constexpr float DRIVE_I = 0;
    constexpr float DRIVE_D = 100;
    constexpr float TURN_P = 9.75;
    constexpr float TURN_I = 0;
    constexpr float TURN_D = 43;

    constexpr float DRIVE_WHEEL_DIAMETER = 3.25;
    constexpr float DRIVE_GEAR_RATIO = (36.0 / 48.0);

    constexpr float TRACK_WIDTH = 11;
    constexpr float ONE_SIDE = (TRACK_WIDTH / 2.0);

    constexpr float INERTIAL_SCALE = (4680.0 / 4657.26);
    // constexpr float INERTIAL_SCALE_2 = (7200.0 / 7061.0) * (4680.0 / 4750.0);

    constexpr float CATA_MATCH_LOAD_SPEED = 120.0 / 2.0;
}
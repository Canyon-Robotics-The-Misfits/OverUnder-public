#include "lib15442c/path/path.hpp"
#include "lib15442c/path/motion_profile.hpp"
#include "lib15442c/logger.hpp"
#include "lib15442c/util/angles.hpp"
#include "lib15442c/util/math.hpp"
#include <iostream>

#define LOGGER "Pure Pursuit"

#include <math.h>

using namespace lib15442c;

std::vector<std::vector<float>> createPolyMatrix(float dist, Point start_point, float start_yaw, float start_vel, float start_accel, Point end_point, float end_yaw, float end_vel, float end_accel, float max_vel, float max_accel, float maxJerk, float maxAngleChange)
{
    float Vxi = start_vel * cos(start_yaw);
    float Vxf = end_vel * cos(end_yaw);
    float Vyi = start_vel * sin(start_yaw);
    float Vyf = end_vel * sin(end_yaw);

    float Axi = start_accel * cos(start_yaw);
    float Axf = end_accel * cos(end_yaw);
    float Ayi = start_accel * sin(start_yaw);
    float Ayf = end_accel * sin(end_yaw);

    float ax0 = start_point.x;
    float ay0 = start_point.y;

    float ax1 = Vxi;
    float ay1 = Vyi;

    float ax2 = Axi / 2;
    float ay2 = Ayi / 2;

    std::vector<std::vector<float>> A = {
        { static_cast<float>(pow(dist, 3)), static_cast<float>(pow(dist, 4)), static_cast<float>(pow(dist, 5)) },
        { 3 * static_cast<float>(pow(dist, 2)), 4 * static_cast<float>(pow(dist, 3)), 5 * static_cast<float>(pow(dist, 4)) },
        { 6 * dist, 12 * static_cast<float>(pow(dist, 2)), 20 * static_cast<float>(pow(dist, 3)) }
    };

    // Resulting Matrix
    std::vector<float> Bx = {
        end_point.x - ax0 - ax1 * dist - ax2 * static_cast<float>(pow(dist, 2)),
        Vxf - ax1 - Axi * dist,
        Axf - Axi
    };

    std::vector<float> By = {
        end_point.y - ay0 - ay1 * dist - ay2 * static_cast<float>(pow(dist, 2)),
        Vyf - ay1 - Ayi * dist,
        Ayf - Ayi
    };

    std::vector<float> Cx = multiplyMatrix(inverseMatrix(A), Bx);
    std::vector<float> Cy = multiplyMatrix(inverseMatrix(A), By);

    std::vector<float> xPoly = { ax0, ax1, ax2, Cx[0], Cx[1], Cx[2] };
    std::vector<float> yPoly = { ay0, ay1, ay2, Cy[0], Cy[1], Cy[2] };

    std::vector<std::vector<float>> polyMatrix = { xPoly, yPoly };

    return polyMatrix;
}

MotionProfile::MotionProfile(Point start_point, float start_yaw, float start_vel, float start_accel, Point end_point, float end_yaw, float end_vel, float end_accel, float max_vel, float max_accel, float maxJerk, float maxAngleChange, float look_ahead):
    Path(start_point, start_yaw, start_vel, look_ahead)
{
    // Convert the angles from degrees to radians
    start_yaw *= PI / 180.0;
    end_yaw *= PI / 180.0;
    maxAngleChange *= PI / 180.0;
    // Convert the start and end yaw to the correct format
    start_yaw = -start_yaw + PI / 2.0;
    end_yaw = -end_yaw + PI / 2.0;

    float S = 0.02;
    for (S = 0.02; S < 20; S += 0.02) {
        std::vector<std::vector<float>> polyMatrix = createPolyMatrix(S, start_point, start_yaw, start_vel, start_accel, end_point, end_yaw, end_vel, end_accel, max_vel, max_accel, maxJerk, maxAngleChange);
        path = {};

        float ds = 0.02;
        float endLoop = false;
        for (float s = 0; s < S; s += ds) {
            MotionProfileState current_state(0, 0, 0, 0, 0, 0, 0, 0);

            // Update the look ahead
            current_state.look_ahead = look_ahead;

            // Get the position
            current_state.x = calculatePoint(polyMatrix[0], s);
            current_state.y = calculatePoint(polyMatrix[1], s);

            // Make sure the point is inside the field
            if (current_state.x >= 144 || current_state.x <= 0 || current_state.y >= 144 || current_state.y <= 0) {
                endLoop = true;
                break;
            }

            // Get the velocity
            float vx = calculateFirstDerivative(polyMatrix[0], s);
            float vy = calculateFirstDerivative(polyMatrix[1], s);
            float vH = sqrt(pow(vx, 2) + pow(vy, 2));
            if (fabs(vH) > max_vel) {
                endLoop = true;
                break;
            }
            current_state.velocity = vH;

            // Get the orientation
            current_state.rotation = atan2(vy, vx);
            if (path.size() > 1) {
                float yawCd0 = current_state.rotation - path[path.size() - 1].rotation;
                if (fabs(yawCd0 / ds) > maxAngleChange) {
                    endLoop = true;
                    break;
                }
                current_state.rotation_speed = yawCd0 / ds;
            } else {
                current_state.rotation_speed = 0;
            }

            // Get the acceleration
            float ax = calculateSecondDerivative(polyMatrix[0], s);
            float ay = calculateSecondDerivative(polyMatrix[1], s);

            float aC0 = sqrt(pow(ax, 2) + pow(ay, 2));
            if (fabs(aC0) > max_accel) {
                endLoop = true;
                break;
            }
            if (path.size() >= 2 && current_state.velocity - path[path.size() - 1].velocity < 0) {
                current_state.acceleration = -aC0;
            } else {
                current_state.acceleration = aC0;
            }

            // Get the jerk
            float jx = calculateThirdDerivative(polyMatrix[0], s);
            float jy = calculateThirdDerivative(polyMatrix[1], s);

            float jC0 = sqrt(pow(jx, 2) + pow(jy, 2));
            if (fabs(jC0) > maxJerk) {
                endLoop = true;
                break;
            }
            if (path.size() >= 2 && current_state.acceleration - path[path.size() - 1].acceleration < 0) {
                current_state.jerk = -jC0;
            } else {
                current_state.jerk = jC0;
            }

            path.push_back(current_state);
        }
        if (!endLoop) {
            break;
        }
    }
}

MotionProfile::MotionProfile(std::vector<Point> points, std::vector<float> yaws, std::vector<float> velocities, std::vector<float> accelerations, std::vector<float> look_aheads, float max_velocity, float max_acceleration, float max_jerk, float max_angle_change):
    Path(points[0], yaws[0], velocities[0], look_aheads[0])
{
    for (int i = 0; i < (int)points.size() - 1; i++) {
        INFO("Creating motion profile %i", i + 1);
        MotionProfile new_profile = MotionProfile(points[i], yaws[i], velocities[i], accelerations[i], points[i + 1], yaws[i + 1], velocities[i + 1], accelerations[i + 1], max_velocity, max_acceleration, max_jerk, max_angle_change, look_aheads[i]);
        INFO("Created motion profile %i!", i + 1);

        for (int j = 0; j < (int)new_profile.path.size(); j++) {
            path.push_back(new_profile.path[j]);
        }
    }
}
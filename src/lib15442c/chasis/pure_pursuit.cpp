#include "lib15442c/logger.hpp"
#include "lib15442c/path/path.hpp"
#include "lib15442c/path/motion_profile.hpp"
#include "lib15442c/chasis/drive_controller.hpp"
#include "lib15442c/path/point.hpp"
#include "lib15442c/util/angles.hpp"
#include "lib15442c/util/math.hpp"
#include <math.h>

#define LOGGER "Pure Pursuit"

float cot(float angle)
{
    float t = tan(angle);
    if (t == 0)
        return 9999999 * lib15442c::sgn(lib15442c::wrap_angle_180_180(angle));
    return 1 / t;
}

lib15442c::Point line_circle_intersection(lib15442c::Point pos1, lib15442c::Point pos2, lib15442c::Point circle_pos, float radius)
{
    // Setup solution variables
    lib15442c::Point solution1 = pos(NAN, NAN);
    lib15442c::Point solution2 = pos(NAN, NAN);

    // Offset line by circle position
    lib15442c::Point offset1 = pos1 - circle_pos;
    lib15442c::Point offset2 = pos2 - circle_pos;

    // Calculate the discriminent (if < 0 no intersection, if = 0 tangent line, and if > 0 secant line)
    float dx = offset2.x - offset1.x;
    float dy = offset2.y - offset1.y;
    float dr_squared = pow(dx, 2) + pow(dy, 2);
    float D = offset1.x * offset2.y - offset2.x * offset1.y;
    float discriminant = pow(radius, 2) * dr_squared - pow(D, 2);

    // if discriminant is >= 0, then solutions exist
    if (discriminant >= 0) {
        float sqrt_discriminant = sqrtf(discriminant);
        // calculate the solutions
        solution1.x = (D * dy + lib15442c::sgn(dy) * dx * sqrt_discriminant) / dr_squared;
        solution2.x = (D * dy - lib15442c::sgn(dy) * dx * sqrt_discriminant) / dr_squared;
        solution1.y = (-D * dx + abs(dy) * sqrt_discriminant) / dr_squared;
        solution2.y = (-D * dx - abs(dy) * sqrt_discriminant) / dr_squared;

        // add currentX and currentY back to the solutions, offset the line back to its original position
        solution1 = solution1 + circle_pos;
        solution2 = solution2 + circle_pos;

        // Make sure solutions are on the line segment
        if (solution1.x < std::min(pos1.x, pos2.x) || solution1.x > std::max(pos1.x, pos2.x) || solution1.y < std::min(pos1.y, pos2.y) || solution1.y > std::max(pos1.y, pos2.y)) {
            solution1 = pos(NAN, NAN);
        }
        if (solution2.x < std::min(pos1.x, pos2.x) || solution2.x > std::max(pos1.x, pos2.x) || solution2.y < std::min(pos1.y, pos2.y) || solution2.y > std::max(pos1.y, pos2.y)) {
            solution2 = pos(NAN, NAN);
        }

        // If one of the solutions doesnt exist, return the other one
        if (isnan(solution2.x) && !isnan(solution1.x)) {
            return solution1;
        } else if (isnan(solution1.x) && !isnan(solution2.x)) {
            return solution2;
        }
        if (isnan(solution1.x) && isnan(solution2.x)) {
            return pos(NAN, NAN);
        }

        // If both points exist, return the one closest to the end of the segment
        if (distance_between(solution1, pos2) < distance_between(solution2, pos2)) {
            return solution1;
        } else {
            return solution2;
        }
    } else {
        // If no solution return that there is no solution
        return pos(NAN, NAN);
    }
}

void lib15442c::DriveController::followPath(lib15442c::Path path, bool reverse, bool async)
{
    async_mutex.lock();
    if (async) {
        pros::Task([this, path, reverse] {
            followPath(path, reverse, false);
        });
        async_mutex.unlock();
        return;
    }

    INFO_TEXT("Start Pure Pursuit...");

    if (path.path.size() == 0) {
        ERROR_TEXT("Path is empty! Skipping path following");
        return;
    }

    // Save the last segment which a point was found on in order to not check any points after
    int last_found_segment = 1;
    int nearest_point_id = 0;

    auto comp_status = pros::competition::get_status();

    while (pros::competition::get_status() == comp_status) {
        // The robot's current position
        lib15442c::Point current_pos = pos(odometry->getX(), odometry->getY());
        lib15442c::Point intersection = path.path[0];

        float look_ahead = path.path[nearest_point_id].look_ahead;

        if (distance_between(current_pos, path.path.back()) <= 1.25 || nearest_point_id == (int)path.path.size() - 1) {
            break;
        }

        // End the task if stopFlag is set
        if (shouldStop()) {
            INFO_TEXT("Stopping Pure Pursuit!");
            break;
        }

        if (distance_between(current_pos, path.path.back()) <= look_ahead) {
            // The points that make up the line segment
            lib15442c::Point pos1 = path.path.back();
            lib15442c::Point pos2 = path.path.back() + pos(144 * cos(path.path.back().rotation), 144 * sin(path.path.back().rotation));

            // Calculate the point to follow
            intersection = line_circle_intersection(pos1, pos2, current_pos, look_ahead);
        } else {
            // Iterate through all of the line segments on the path to find intersections
            for (int i = last_found_segment; i < (int)path.path.size() - 1; i++) {
                // The points that make up the line segment
                lib15442c::Point pos1 = path.path[i];
                lib15442c::Point pos2 = path.path[i + 1];

                // Calculate the point to follow
                lib15442c::Point temp = line_circle_intersection(pos1, pos2, current_pos, look_ahead);

                // If the point exists and is in the right direction, then set the target point to it and exit the loop
                if (!isnan(temp.x) && (distance_between(temp, pos2) < distance_between(current_pos, pos2))) {
                    intersection = temp;
                    last_found_segment = i;
                    break;
                }
            }
        }        

        int closest_id = -1;
        float closest_distance = INFINITY;
        for (int i = std::max(0, nearest_point_id - 20); i < std::min(nearest_point_id + 20, (int)path.path.size()); i++) {
            float distance = distance_between(current_pos, path.path[i]);
            if (distance < closest_distance) {
                closest_distance = distance;
                closest_id = i;
            }
        }
        nearest_point_id = closest_id;

        // Calculate the speed
        float linear_speed = path.path[nearest_point_id].velocity;
        if (fabs(linear_speed) < 8) {
            linear_speed = 8 * lib15442c::sgn(linear_speed);
        }

        float robot_angle = odometry->getRotation() + (reverse ? 180 : 0);
        float angleError = angle_error(robot_angle, angle_between(current_pos, intersection));
        angleError = rad(angleError);
        float rotational_speed = drivetrain->get_track_width() * sin(angleError) * linear_speed / distance_between(intersection, current_pos);

        // std::cout << current_pos.x << ", " << current_pos.y << ", " << intersection.x << ", " << intersection.y << ", " << linear_speed << ", " << rotational_speed << std::endl;
        // std::cout << pros::millis() << ", " << linear_speed * (127.0 / 57.0) << ", " << rotational_speed * 0.249477 * 0.35 << ", " << std::endl;

        drivetrain->move_ratio(linear_speed * (127.0 / 57.0) * (reverse ? -1 : 1), rotational_speed * -6.75 * (reverse ? -1 : 1));

        // Delay to allow other tasks to run
        pros::delay(20);
    }

    // Stop the drivetrain
    drivetrain->move(0, 0);

    async_mutex.unlock();
    
    INFO_TEXT("Pure Pursuit Finished!");
}
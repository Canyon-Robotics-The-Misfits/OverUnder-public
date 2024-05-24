#include <algorithm>
#include <math.h>

#include "lib15442c/util/angles.hpp"
#include "lib15442c/util/math.hpp"
#include "path.hpp"

using namespace lib15442c;

PathState::PathState(Point point, float rotation, float velocity, float look_ahead) :
    Point(point.x, point.y), rotation(rotation), velocity(velocity), look_ahead(look_ahead) {};

Path::Path(Point start_point, float start_angle, float start_speed, float start_lookahead)
{
    append_state(PathState(start_point, start_angle, start_speed, start_lookahead));
}

void Path::append_state(PathState segment)
{
    path.push_back(segment);
}

void Path::append_line(Point point, float end_speed, float end_look_ahead, float point_spacing)
{
    float start_speed = path.back().velocity;
    if (end_speed == -1)
        end_speed = start_speed;

    float start_look_ahead = path.back().look_ahead;
    if (end_look_ahead == -1)
        end_look_ahead = start_look_ahead;

    Point start_point = path.back();

    float segment_count = ceil(distance_between(point, start_point) / point_spacing);

    float angle = angle_between(start_point, point);

    for (float t = 1/segment_count; t <= 1; t += 1/segment_count) {
        append_state(PathState(
            pos(lerp(start_point.x, point.x, t), lerp(start_point.y, point.y, t)), angle,
            lerp(start_speed, end_speed, t), lerp(start_look_ahead, end_look_ahead, t))
        );
    }
}

float lerp_bezier(float v1, float v2, float v3, float v4, float t)
{
    return pow((1 - t), 3) * v1 + 3 * pow((1 - t), 2) * t * v2 + 3 * (1 - t) * pow(t, 2) * v3 + pow(t, 3) * v4;
}

float bezier_derivative(float v1, float v2, float v3, float v4, float t)
{
    // return -3 * (v1 * pow((1 - t), 2) + v2 * (-3 * pow(t, 2) + 4 * t - 1) + t * (3 * v3 * t - 2 * v3 - v4 * t));
    return -3 * pow(1 - t, 2) * v1 + 3 * pow(1 - t, 2) * v2 - 6 * t * (1 - t) * v2 - 3 * pow(t, 2) * v3 + 6 * t * (1 - t) * v3 + 3 * pow(t, 2) * v4;

}

float bezier_angle(Point pos1, Point pos2, Point pos3, Point pos4, float t)
{
    return deg(atan2(bezier_derivative(pos1.y, pos2.y, pos3.y, pos4.y, t), bezier_derivative(pos1.x, pos2.x, pos3.x, pos4.x, t)));
}

float bezier_curvature(float x, float y, Point pos1, Point pos2, Point pos3, Point pos4, float t) {    
    Point start_pos = pos(x, y);
    Point end_pos = pos(lerp_bezier(pos1.x, pos2.x, pos3.x, pos4.x, t + 0.001), lerp_bezier(pos1.y, pos2.y, pos3.y, pos4.y, t + 0.001));

    float start_angle = bezier_angle(pos1, pos2, pos3, pos4, t);
    float end_angle = bezier_angle(pos1, pos2, pos3, pos4, t + 0.001);

    return (start_angle - end_angle) / distance_between(start_pos, end_pos);
}

void Path::append_bezier(Point pos1, Point pos2, Point pos3, Point pos4, float end_speed, float look_ahead, float resolution, float curvature_multiplier)
{
    float start_speed = path.back().velocity;
    if (end_speed == -1)
        end_speed = start_speed;

    float start_look_ahead = path.back().look_ahead;
    if (look_ahead == -1)
        look_ahead = start_look_ahead;

    float t_change = 1 / resolution;

    for (float t = 0; t < 1; t += t_change) {
        float x = lerp_bezier(pos1.x, pos2.x, pos3.x, pos4.x, t);
        float y = lerp_bezier(pos1.y, pos2.y, pos3.y, pos4.y, t);

        float angle = bezier_angle(pos1, pos2, pos3, pos4, t);

        // float curvature = bezier_curvature(x, y, pos1, pos2, pos3, pos4, t);

        float speed = lerp(start_speed, end_speed, t);

        append_state(PathState(pos(x, y), angle, speed, lerp(start_look_ahead, look_ahead, t)));
    }

    append_line(pos4, end_speed, look_ahead);
}
#pragma once

#include <vector>

#include "point.hpp"

namespace lib15442c
{
    /**
     * @brief A point on a path
     */
    class PathState : public Point
    {
    public:
        float rotation;
        float velocity;
        float look_ahead;

        PathState(Point point, float rotation, float velocity, float look_ahead);
    };

    /**
     * @brief A path to be followed with pure pursuit or a motion profile
     */
    class Path
    {
    private:
        /**
         * @brief Add a PathState to the end of the path
         *
         * @param path_state The state to append
         */
        void append_state(PathState path_state);

    public:
        std::vector<PathState> path;

        /**
         * @brief Create a new path
         *
         * @param start_point The first point on the path
         * @param start_angle The starting angle of the path
         * @param start_speed The speed to start going at
         * @param start_look_ahead The look ahead to use on the begining of the path
         */
        Path(Point start_point, float start_angle, float start_speed, float start_look_ahead);

        /**
         * @brief Add a line to the end of the path which leads to a Vector
         *
         * @param point The Vector to aim the line at
         * @param end_speed The speed to end at when following the line
         * @param end_look_ahead The look ahead to be at at the end of the line
         * @param line_spacing The distance between each point on the line
         */
        void append_line(Point point, float end_speed = -1, float end_look_ahead = -1, float line_spacing = 2);

        /**
         * @brief Add a Bezier Curve to the end of the path
         *
         * @param pos1 The start point of the path
         * @param pos2 The first control point
         * @param pos3 The second control point
         * @param pos4 The end point of the path
         * @param endspeed The speed to end this part of the path at
         * @param end_look_ahead The look ahead to end at when following the bezier
         * @param resolution The number of line segments to split the bezier into
         * @param curvature_multiplier The multiplier for the curvature to calculate speed
         */
        void append_bezier(Point pos1, Point pos2, Point pos3, Point pos4, float end_speed = -1, float end_look_ahead = -1, float resolution = 15, float curvature_multiplier = .5);
    };
} // namespace lib15442c

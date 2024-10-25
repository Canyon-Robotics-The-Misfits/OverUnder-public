#pragma once
#include "lib15442c/path/point.hpp"
#include <vector>

namespace lib15442c
{
    /**
     * @brief Get the sign of a number
     *
     * @param value The number to get the sign of
     * @return float -1 for negative and 1 for positive and zero
     */
    float sgn(float value);

    /**
     * @brief Average all the values in a vector
     *
     * @param v The vector
     * @return float The average of the values
     */
    float average_vector(std::vector<float> v);
    /**
     * @brief Average all the values in a vector
     *
     * @param v The vector
     * @return double The average of the values
     */
    double average_vector(std::vector<double> v);

    /**
     * @brief Get the angle between two points
     *
     * @param x1 The x of point 1
     * @param y1 The y of point 1
     * @param x2 The x of point 2
     * @param y2 The y of point 2
     * @return float The angle between the points
     */
    float angle_between(float x1, float y1, float x2, float y2);
    /**
     * @brief Get the angle between two points
     *
     * @param pos1 The first point
     * @param pos2 The second point
     * @return float The angle between the points
     */
    float angle_between(Point pos1, Point pos2);

    /**
     * @brief Get the distance between two points
     *
     * @param x1 The x of point 1
     * @param y1 The y of point 1
     * @param x2 The x of point 2
     * @param y2 The y of point 2
     * @return float The distance between the points
     */
    float distance_between(float x1, float y1, float x2, float y2);
    /**
     * @brief Get the distance between two points
     *
     * @param pos1 The first point
     * @param pos2 The second point
     * @return float The distance between the points
     */
    float distance_between(Point pos1, Point pos2);

    // Get the inverse of a 3x3 matrix
    std::vector<std::vector<float>> inverseMatrix(std::vector<std::vector<float>> matrix);

    // Multiply a 3x3 matrix by a 1x3 matrix
    std::vector<float> multiplyMatrix(std::vector<std::vector<float>> matrix, std::vector<float> vector);

    // Calulcate the point on a polynomial at a given t
    float calculatePoint(std::vector<float> poly, float t);

    // Calculate the derivative of a polynomial at a given t
    float calculateFirstDerivative(std::vector<float> poly, float t);

    // Calculate the second derivative of a polynomial at a given t
    float calculateSecondDerivative(std::vector<float> poly, float t);

    // Calculate the third derivative of a polynomial at a given t
    float calculateThirdDerivative(std::vector<float> poly, float t);
} // namespace util

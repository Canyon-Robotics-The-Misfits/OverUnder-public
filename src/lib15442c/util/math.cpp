#include "math.hpp"
#include "lib15442c/path/point.hpp"
#include "lib15442c/util/angles.hpp"
#include <math.h>
#include <numeric>

using namespace lib15442c;

float lib15442c::sgn(float value)
{
    return value >= 0 ? 1 : -1;
}

float lib15442c::average_vector(std::vector<float> v)
{
    return std::accumulate(v.begin(), v.end(), 0.0) / v.size();
}

double lib15442c::average_vector(std::vector<double> v)
{
    return std::accumulate(v.begin(), v.end(), 0.0) / v.size();
}

float lib15442c::angle_between(float x1, float y1, float x2, float y2)
{
    return atan((x1 - x2) / (y1 - y2)) * 180 / PI;
}
float lib15442c::angle_between(Point pos1, Point pos2)
{
    return angle_between(pos1.x, pos1.y, pos2.x, pos2.y);
}

float lib15442c::distance_between(float x1, float y1, float x2, float y2)
{
    return sqrt((x1 - x2) * (x1 - x2) + (y1 - y2) * (y1 - y2));
}
float lib15442c::distance_between(Point pos1, Point pos2)
{
    return distance_between(pos1.x, pos1.y, pos2.x, pos2.y);
}

std::vector<float> lib15442c::multiplyMatrix(std::vector<std::vector<float>> matrix, std::vector<float> vector)
{
    std::vector<float> result = { 0, 0, 0 };
    result[0] = matrix[0][0] * vector[0] + matrix[0][1] * vector[1] + matrix[0][2] * vector[2];
    result[1] = matrix[1][0] * vector[0] + matrix[1][1] * vector[1] + matrix[1][2] * vector[2];
    result[2] = matrix[2][0] * vector[0] + matrix[2][1] * vector[1] + matrix[2][2] * vector[2];
    return result;
}

std::vector<std::vector<float>> lib15442c::inverseMatrix(std::vector<std::vector<float>> matrix)
{
    // Get the determinant of the matrix
    float d1 = matrix[0][0] * matrix[1][1] * matrix[2][2] + matrix[0][1] * matrix[1][2] * matrix[2][0] + matrix[0][2] * matrix[1][0] * matrix[2][1];
    float d2 = matrix[0][2] * matrix[1][1] * matrix[2][0] + matrix[0][0] * matrix[1][2] * matrix[2][1] + matrix[0][1] * matrix[1][0] * matrix[2][2];
    float det = d1 - d2;
    // Get the 2x2 matrix minors
    float n[3][3];
    for (int i = 0; i < 3; i++) {
        for (int j = 0; j < 3; j++) {
            float m[4];
            int idx = 0;
            for (int k = 0; k < 3; k++) {
                if (k != i) {
                    for (int l = 0; l < 3; l++) {
                        if (l != j) {
                            m[idx++] = matrix[k][l];
                        }
                    }
                }
            }
            n[i][j] = m[0] * m[3] - m[1] * m[2];
        }
    }
    // Alternate the signs of the cofactors
    for (int i = 0; i < 3; i++) {
        for (int j = 0; j < 3; j++) {
            n[i][j] *= (i + j) % 2 == 1 ? -1 : 1;
        }
    }
    // Reflect the matrix across the diagonal and divide by the determinant
    std::vector<std::vector<float>> result = { { 0, 0, 0 }, { 0, 0, 0 }, { 0, 0, 0 } };

    for (int i = 0; i < 3; i++) {
        for (int j = 0; j < 3; j++) {
            result[i][j] = n[j][i] / det;
        }
    }
    // Return the inverse matrix
    return result;
}
float lib15442c::calculatePoint(std::vector<float> poly, float t)
{
    return poly[0] + poly[1] * t + poly[2] * pow(t, 2) + poly[3] * pow(t, 3) + poly[4] * pow(t, 4) + poly[5] * pow(t, 5);
}

float lib15442c::calculateFirstDerivative(std::vector<float> poly, float t)
{
    return poly[1] + 2 * poly[2] * t + 3 * poly[3] * pow(t, 2) + 4 * poly[4] * pow(t, 3) + 5 * poly[5] * pow(t, 4);
}

float lib15442c::calculateSecondDerivative(std::vector<float> poly, float t)
{
    return 2 * poly[2] + 6 * poly[3] * t + 12 * poly[4] * pow(t, 2) + 20 * poly[5] * pow(t, 3);
}

float lib15442c::calculateThirdDerivative(std::vector<float> poly, float t)
{
    return 6 * poly[3] + 24 * poly[4] * t + 60 * poly[5] * pow(t, 2);
}
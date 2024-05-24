#pragma once
#include "math.hpp"

namespace lib15442c {
/**
 * @brief The mathmatical constant PI
 */
constexpr float PI = 3.141592653589793238462643;

/**
 * @brief A macro to convert angles in degrees to angles in radians
 */
#define rad(angle) ((angle)*PI / 180)
/**
 * @brief A macro to convert angles in radians to angles in degrees
 */
#define deg(angle) ((angle) / PI * 180)

/**
 * @brief Get the error between angle a and angle b, accounting for angle wrapping
 *
 * @param angle_a The first angle
 * @param angle_b The second angle
 * @return float The error between the two angles
 */
float angle_error(float angle_a, float angle_b);

/**
 * @brief Wraps an angle in the range of 180 to -180
 *
 * @param angle The angle to wrap
 * @return float The wrapped angle
 */
float wrap_angle_180_180(float angle);

/**
 * @brief Wraps an angle in the range of 0 to 360
 *
 * @param angle The angle to wrap
 * @return float The wrapped angle
 */
float wrap_angle_0_360(float angle);
} // namespace lib15442c
#pragma once

namespace libmavnetics {

/**
 * @brief AngularDirection
 *
 * When turning, the user may want to specify the direction the robot should
 * turn in. This enum class has 3 values: CW_CLOCKWISE, CCW_COUNTERCLOCKWISE,
 * and AUTO AUTO will make the robot turn in the shortest direction, and will be
 * the most used value
 */
enum class AngularDirection {
  CW_CLOCKWISE,         /** turn clockwise */
  CCW_COUNTERCLOCKWISE, /** turn counter-clockwise */
  AUTO /** turn in the direction with the shortest distance to target */
};

/**
 * @brief Return the sign of a number
 *
 * @param x the number to get the sign of
 * @return int - -1 if negative, 1 if positive
 *
 * @b Example
 * @code {.cpp}
 * sgn(-10); // returns -1
 * sgn(10); // returns 1
 * sgn(0); // returns 1 (by convention)
 * @endcode
 */
template <typename T> constexpr T sgn(T value) { return value < 0 ? -1 : 1; }

/**
 * @brief Sanitize an angle so its positive and within the range of 0 to 2pi or
 * 0 to 360
 *
 * @param angle the angle to sanitize
 * @param radians whether the angle is in radians or no. True by default
 * @return constexpr float
 *
 * @b Example
 * @code {.cpp}
 * // sanitize angle in degrees
 * sanitizeAngle(-90, false); // returns 270
 * sanitizeAngle(370, false); // returns 10
 * // sanitize angle in radians
 * sanitizeAngle(-M_PI, true); // returns pi
 * sanitizeAngle(7 * M_PI, true); // returns pi
 * // you can also use the default value of radians
 * sanitizeAngle(-M_PI); // returns pi
 * sanitizeAngle(7 * M_PI); // returns pi
 * @endcode
 */
constexpr float sanitizeAngle(float angle, bool radians = true);

/**
 * @brief Calculate the error between 2 angles. Useful when calculating the
 * error between 2 headings
 *
 * @param target target angle
 * @param position position angle
 * @param radians true if angle is in radians, false if not. Radians by default
 * @param direction which direction to turn to get to the target angle
 * @return float wrapped angle
 *
 * @b Example
 * @code {.cpp}
 * angleError(10, 350, false); // returns 20
 * angleError(350, 10, false); // returns -20
 * @endcode
 */
float angleError(float target, float position, bool radians = true,
                 AngularDirection direction = AngularDirection::AUTO);
} // namespace libmavnetics
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
template <typename T> constexpr T sgn(T& value) { return value < 0 ? -1 : 1; }

/**
 * @brief Linearly interpolates a value from start to end at time t
 *
 * @param start start value to interpolate from
 * @param end end value to interpolate to
 * @param t number between 0-1 signifying where in the interplation to sample from
 * @return the interpolated value
 */
template<typename T>
constexpr T lerp(T& start, T& end, double t) {
    return start + (end - start) * t;
}

/**
 * Returns modulus of input.
 *
 * @param input        Input value to wrap.
 * @param minimumInput The minimum value expected from the input.
 * @param maximumInput The maximum value expected from the input.
 */
template <typename T>
constexpr T inputModulus(T input, T minimumInput, T maximumInput) {
  T modulus = maximumInput - minimumInput;

  int numMax = (input - minimumInput) / modulus;
  input -= numMax * modulus;

  int numMin = (input - maximumInput) / modulus;
  input -= numMin * modulus;

  return input;
}

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

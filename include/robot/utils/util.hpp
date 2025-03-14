#pragma once

namespace libmavnetics {
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

} // namespace libmavnetics
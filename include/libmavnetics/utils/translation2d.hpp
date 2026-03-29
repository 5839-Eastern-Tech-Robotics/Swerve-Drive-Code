// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <algorithm>
#include <initializer_list>
#include <span>

#include "Eigen/Core"
#include "libmavnetics/utils/rotation2d.hpp"
#include "units/area.h"
#include "units/length.h"
#include "units/math.h"

namespace libmavnetics {

/**
 * Represents a translation in 2D space.
 * This object can be used to represent a point or a vector.
 *
 * This assumes that you are using conventional mathematical axes.
 * When the robot is at the origin facing in the positive X direction, forward
 * is positive X and left is positive Y.
 */
class Translation2D {
public:
  /**
   * Constructs a Translation2D with X and Y components equal to zero.
   */
  constexpr Translation2D() = default;

  /**
   * Constructs a Translation2D with the X and Y components equal to the
   * provided values.
   *
   * @param x The x component of the translation.
   * @param y The y component of the translation.
   */
  constexpr Translation2D(units::meter_t x, units::meter_t y)
      : m_x{x}, m_y{y} {}

  /**
   * Constructs a Translation2D with the provided distance and angle. This is
   * essentially converting from polar coordinates to Cartesian coordinates.
   *
   * @param distance The distance from the origin to the end of the translation.
   * @param angle The angle between the x-axis and the translation vector.
   */
  constexpr Translation2D(units::meter_t distance, const Rotation2D &angle)
      : m_x{distance * angle.cos()}, m_y{distance * angle.sin()} {}

  /**
   * Calculates the distance between two translations in 2D space.
   *
   * The distance between translations is defined as √((x₂−x₁)²+(y₂−y₁)²).
   *
   * @param other The translation to compute the distance to.
   *
   * @return The distance between the two translations.
   */
  constexpr units::meter_t distance(const Translation2D &other) const {
    return units::math::hypot(other.m_x - m_x, other.m_y - m_y);
  }

  /**
   * Calculates the square of the distance between two translations in 2D space.
   * This is equivalent to squaring the result of Distance(Translation2D), but
   * avoids computing a square root.
   *
   * The square of the distance between translations is defined as
   * (x₂−x₁)²+(y₂−y₁)².
   *
   * @param other The translation to compute the squared distance to.
   * @return The square of the distance between the two translations.
   */
  constexpr units::square_meter_t
  squaredDistance(const Translation2D &other) const {
    return units::math::pow<2>(other.m_x - m_x) +
           units::math::pow<2>(other.m_y - m_y);
  }

  /**
   * Returns the X component of the translation.
   *
   * @return The X component of the translation.
   */
  constexpr units::meter_t x() const { return m_x; }

  /**
   * Returns the Y component of the translation.
   *
   * @return The Y component of the translation.
   */
  constexpr units::meter_t y() const { return m_y; }

  /**
   * Returns the norm, or distance from the origin to the translation.
   *
   * @return The norm of the translation.
   */
  constexpr units::meter_t norm() const { return units::math::hypot(m_x, m_y); }

  /**
   * Returns the squared norm, or squared distance from the origin to the
   * translation. This is equivalent to squaring the result of Norm(), but
   * avoids computing a square root.
   *
   * @return The squared norm of the translation.
   */
  constexpr units::square_meter_t squaredNorm() const {
    return units::math::pow<2>(m_x) + units::math::pow<2>(m_y);
  }

  /**
   * Returns the angle this translation forms with the positive X axis.
   *
   * @return The angle of the translation
   */
  constexpr Rotation2D angle() const {
    return Rotation2D{m_x.value(), m_y.value()};
  }

  /**
   * Applies a rotation to the translation in 2D space.
   *
   * This multiplies the translation vector by a counterclockwise rotation
   * matrix of the given angle.
   *
   * <pre>
   * [x_new]   [other.cos, -other.sin][x]
   * [y_new] = [other.sin,  other.cos][y]
   * </pre>
   *
   * For example, rotating a Translation2D of &lt;2, 0&gt; by 90 degrees will
   * return a Translation2D of &lt;0, 2&gt;.
   *
   * @param other The rotation to rotate the translation by.
   *
   * @return The new rotated translation.
   */
  constexpr Translation2D rotateBy(const Rotation2D &other) const {
    return {m_x * other.cos() - m_y * other.sin(),
            m_x * other.sin() + m_y * other.cos()};
  }

  /**
   * Rotates this translation around another translation in 2D space.
   *
   * <pre>
   * [x_new]   [rot.cos, -rot.sin][x - other.x]   [other.x]
   * [y_new] = [rot.sin,  rot.cos][y - other.y] + [other.y]
   * </pre>
   *
   * @param other The other translation to rotate around.
   * @param rot The rotation to rotate the translation by.
   * @return The new rotated translation.
   */
  constexpr Translation2D rotateAround(const Translation2D &other,
                                       const Rotation2D &rot) const {
    return {(m_x - other.x()) * rot.cos() - (m_y - other.y()) * rot.sin() +
                other.x(),
            (m_x - other.x()) * rot.sin() + (m_y - other.y()) * rot.cos() +
                other.y()};
  }

  /**
   * Computes the dot product between this translation and another translation
   * in 2D space.
   *
   * The dot product between two translations is defined as x₁x₂+y₁y₂.
   *
   * @param other The translation to compute the dot product with.
   * @return The dot product between the two translations.
   */
  constexpr units::square_meter_t dot(const Translation2D &other) const {
    return m_x * other.x() + m_y * other.y();
  }

  /**
   * Computes the cross product between this translation and another translation
   * in 2D space.
   *
   * The 2D cross product between two translations is defined as x₁y₂-x₂y₁.
   *
   * @param other The translation to compute the cross product with.
   * @return The cross product between the two translations.
   */
  constexpr units::square_meter_t cross(const Translation2D &other) const {
    return m_x * other.y() - m_y * other.x();
  }

  /**
   * Returns the sum of two translations in 2D space.
   *
   * For example, Translation3d{1.0, 2.5} + Translation3d{2.0, 5.5} =
   * Translation3d{3.0, 8.0}.
   *
   * @param other The translation to add.
   *
   * @return The sum of the translations.
   */
  constexpr Translation2D operator+(const Translation2D &other) const {
    return {x() + other.x(), y() + other.y()};
  }

  /**
   * Returns the difference between two translations.
   *
   * For example, Translation2D{5.0, 4.0} - Translation2D{1.0, 2.0} =
   * Translation2D{4.0, 2.0}.
   *
   * @param other The translation to subtract.
   *
   * @return The difference between the two translations.
   */
  constexpr Translation2D operator-(const Translation2D &other) const {
    return *this + -other;
  }

  /**
   * Returns the inverse of the current translation. This is equivalent to
   * rotating by 180 degrees, flipping the point over both axes, or negating all
   * components of the translation.
   *
   * @return The inverse of the current translation.
   */
  constexpr Translation2D operator-() const { return {-m_x, -m_y}; }

  /**
   * Returns the translation multiplied by a scalar.
   *
   * For example, Translation2D{2.0, 2.5} * 2 = Translation2D{4.0, 5.0}.
   *
   * @param scalar The scalar to multiply by.
   *
   * @return The scaled translation.
   */
  constexpr Translation2D operator*(double scalar) const {
    return {scalar * m_x, scalar * m_y};
  }

  /**
   * Returns the translation divided by a scalar.
   *
   * For example, Translation2D{2.0, 2.5} / 2 = Translation2D{1.0, 1.25}.
   *
   * @param scalar The scalar to divide by.
   *
   * @return The scaled translation.
   */
  constexpr Translation2D operator/(double scalar) const {
    return operator*(1.0 / scalar);
  }

  /**
   * Checks equality between this Translation2D and another object.
   *
   * @param other The other object.
   * @return Whether the two objects are equal.
   */
  constexpr bool operator==(const Translation2D &other) const {
    return units::math::abs(m_x - other.m_x) < 1E-9_m &&
           units::math::abs(m_y - other.m_y) < 1E-9_m;
  }

  /**
   * Returns the nearest Translation2D from a collection of translations
   * @param translations The collection of translations.
   * @return The nearest Translation2D from the collection.
   */
  constexpr Translation2D
  nearest(std::span<const Translation2D> translations) const {
    return *std::min_element(
        translations.begin(), translations.end(),
        [this](const Translation2D &a, const Translation2D &b) {
          return this->distance(a) < this->distance(b);
        });
  }

  /**
   * Returns the nearest Translation2D from a collection of translations
   * @param translations The collection of translations.
   * @return The nearest Translation2D from the collection.
   */
  constexpr Translation2D
  nearest(std::initializer_list<Translation2D> translations) const {
    return *std::min_element(
        translations.begin(), translations.end(),
        [this](const Translation2D &a, const Translation2D &b) {
          return this->distance(a) < this->distance(b);
        });
  }

private:
  units::meter_t m_x = 0_m;
  units::meter_t m_y = 0_m;
};

} // namespace libmavnetics

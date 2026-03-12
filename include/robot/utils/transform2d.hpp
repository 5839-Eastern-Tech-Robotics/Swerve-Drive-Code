// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <utility>

#include "robot/utils/rotation2d.hpp"
#include "robot/utils/translation2d.hpp"

namespace libmavnetics {

class Pose2D;

/**
 * Represents a transformation for a Pose2d in the pose's frame.
 */
class Transform2D {
 public:
  /**
   * Constructs the transform that maps the initial pose to the final pose.
   *
   * @param initial The initial pose for the transformation.
   * @param final The final pose for the transformation.
   */
  constexpr Transform2D(const Pose2D& initial, const Pose2D& final);

  /**
   * Constructs a transform with the given translation and rotation components.
   *
   * @param translation Translational component of the transform.
   * @param rotation Rotational component of the transform.
   */
  constexpr Transform2D(Translation2D translation, Rotation2D rotation)
      : m_translation{std::move(translation)},
        m_rotation{std::move(rotation)} {}

  /**
   * Constructs a transform with x and y translations instead of a separate
   * Translation2d.
   *
   * @param x The x component of the translational component of the transform.
   * @param y The y component of the translational component of the transform.
   * @param rotation The rotational component of the transform.
   */
  constexpr Transform2D(units::meter_t x, units::meter_t y, Rotation2D rotation)
      : m_translation{x, y}, m_rotation{std::move(rotation)} {}

  /**
   * Constructs the identity transform -- maps an initial pose to itself.
   */
  constexpr Transform2D() = default;

  /**
   * Returns the translation component of the transformation.
   *
   * @return Reference to the translational component of the transform.
   */
  constexpr const Translation2D& translation() const { return m_translation; }

  /**
   * Returns the X component of the transformation's translation.
   *
   * @return The x component of the transformation's translation.
   */
  constexpr units::meter_t x() const { return m_translation.x(); }

  /**
   * Returns the Y component of the transformation's translation.
   *
   * @return The y component of the transformation's translation.
   */
  constexpr units::meter_t y() const { return m_translation.y(); }

  /**
   * Returns the rotational component of the transformation.
   *
   * @return Reference to the rotational component of the transform.
   */
  constexpr const Rotation2D& rotation() const { return m_rotation; }

  /**
   * Invert the transformation. This is useful for undoing a transformation.
   *
   * @return The inverted transformation.
   */
  constexpr Transform2D inverse() const {
    // We are rotating the difference between the translations
    // using a clockwise rotation matrix. This transforms the global
    // delta into a local delta (relative to the initial pose).
    return Transform2D{(-translation()).rotateBy(-rotation()), -rotation()};
  }

  /**
   * Multiplies the transform by the scalar.
   *
   * @param scalar The scalar.
   * @return The scaled Transform2D.
   */
  constexpr Transform2D operator*(double scalar) const {
    return Transform2D(m_translation * scalar, m_rotation * scalar);
  }

  /**
   * Divides the transform by the scalar.
   *
   * @param scalar The scalar.
   * @return The scaled Transform2D.
   */
  constexpr Transform2D operator/(double scalar) const {
    return *this * (1.0 / scalar);
  }

  /**
   * Composes two transformations. The second transform is applied relative to
   * the orientation of the first.
   *
   * @param other The transform to compose with this one.
   * @return The composition of the two transformations.
   */
  constexpr Transform2D operator+(const Transform2D& other) const;

  /**
   * Checks equality between this Transform2D and another object.
   */
  constexpr bool operator==(const Transform2D&) const = default;

 private:
  Translation2D m_translation;
  Rotation2D m_rotation;
};

}  // namespace libmavnetics

#include "robot/utils/pose2d.hpp"

namespace libmavnetics {

constexpr Transform2D::Transform2D(const Pose2D& initial, const Pose2D& final) {
  // To transform the global translation delta to be relative to the initial
  // pose, rotate by the inverse of the initial pose's orientation.
  m_translation = (final.translation() - initial.translation())
                      .rotateBy(-initial.rotation());

  m_rotation = final.rotation().relativeTo(initial.rotation());
}

constexpr Transform2D Transform2D::operator+(const Transform2D& other) const {
  return Transform2D{Pose2D{}, Pose2D{}.transformBy(*this).transformBy(other)};
}

}  // namespace libmavnetics

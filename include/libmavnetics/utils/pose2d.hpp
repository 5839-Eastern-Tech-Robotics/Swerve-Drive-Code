// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <utility>

#include "libmavnetics/utils/rotation2d.hpp"
#include "libmavnetics/utils/translation2d.hpp"
#include "units/length.h"

namespace libmavnetics {

/**
 * Represents a 2D pose containing translational and rotational elements.
 */
class Pose2D {
public:
  /**
   * Constructs a pose at the origin facing toward the positive X axis.
   */
  constexpr Pose2D() = default;
  constexpr Pose2D(const Pose2D &) = default;
  constexpr Pose2D &operator=(const Pose2D &) = default;
  constexpr Pose2D(Pose2D &&) = default;
  constexpr Pose2D &operator=(Pose2D &&) = default;

  /**
   * Constructs a pose with the specified translation and rotation.
   *
   * @param translation The translational component of the pose.
   * @param rotation The rotational component of the pose.
   */
  constexpr Pose2D(Translation2D translation, Rotation2D rotation)
      : m_translation{std::move(translation)}, m_rotation{std::move(rotation)} {
  }

  /**
   * Constructs a pose with x and y translations instead of a separate
   * Translation2d.
   *
   * @param x The x component of the translational component of the pose.
   * @param y The y component of the translational component of the pose.
   * @param rotation The rotational component of the pose.
   */
  constexpr Pose2D(units::meter_t x, units::meter_t y, Rotation2D rotation)
      : m_translation{x, y}, m_rotation{std::move(rotation)} {}

  /**
   * Transforms the pose by the given transformation and returns the new
   * transformed pose.
   *
   * <pre>
   * [x_new]    [cos, -sin, 0][transform.x]
   * [y_new] += [sin,  cos, 0][transform.y]
   * [t_new]    [  0,    0, 1][transform.t]
   * </pre>
   *
   * @param other The transform to transform the pose by.
   *
   * @return The transformed pose.
   */
  constexpr Pose2D operator+(const Pose2D &other) const {
    return transformBy(other);
  }

  /**
   * Returns the Transform2d that maps the one pose to another.
   *
   * @param other The initial pose of the transformation.
   * @return The transform that maps the other pose to the current pose.
   */
  constexpr Pose2D operator-(const Pose2D &other) const;

  /**
   * Checks equality between this Pose2D and another object.
   */
  constexpr bool operator==(const Pose2D &) const = default;

  /**
   * Returns the underlying translation.
   *
   * @return Reference to the translational component of the pose.
   */
  constexpr const Translation2D &translation() const { return m_translation; }

  /**
   * Returns the X component of the pose's translation.
   *
   * @return The x component of the pose's translation.
   */
  constexpr units::meter_t x() const { return m_translation.x(); }

  /**
   * Returns the Y component of the pose's translation.
   *
   * @return The y component of the pose's translation.
   */
  constexpr units::meter_t y() const { return m_translation.y(); }

  /**
   * Returns the underlying rotation.
   *
   * @return Reference to the rotational component of the pose.
   */
  constexpr const Rotation2D &rotation() const { return m_rotation; }

  /**
   * Multiplies the current pose by a scalar.
   *
   * @param scalar The scalar.
   *
   * @return The new scaled Pose2D.
   */
  constexpr Pose2D operator*(double scalar) const {
    return Pose2D{m_translation * scalar, m_rotation * scalar};
  }

  /**
   * Divides the current pose by a scalar.
   *
   * @param scalar The scalar.
   *
   * @return The new scaled Pose2D.
   */
  constexpr Pose2D operator/(double scalar) const {
    return *this * (1.0 / scalar);
  }

  /**
   * Rotates the pose around the origin and returns the new pose.
   *
   * @param other The rotation to transform the pose by.
   *
   * @return The rotated pose.
   */
  constexpr Pose2D rotateBy(const Rotation2D &other) const {
    return {m_translation.rotateBy(other), m_rotation.rotateBy(other)};
  }

  /**
   * Transforms the pose by the given transformation and returns the new pose.
   * See + operator for the matrix multiplication performed.
   *
   * @param other The transform to transform the pose by.
   *
   * @return The transformed pose.
   */
  constexpr Pose2D transformBy(const Pose2D &other) const;

  /**
   * Returns the current pose relative to the given pose.
   *
   * This function can often be used for trajectory tracking or pose
   * stabilization algorithms to get the error between the reference and the
   * current pose.
   *
   * @param other The pose that is the origin of the new coordinate frame that
   * the current pose will be converted into.
   *
   * @return The current pose relative to the new origin pose.
   */
  constexpr Pose2D relativeTo(const Pose2D &other) const;

  /**
   * Rotates the current pose around a point in 2D space.
   *
   * @param point The point in 2D space to rotate around.
   * @param rot The rotation to rotate the pose by.
   *
   * @return The new rotated pose.
   */
  constexpr Pose2D rotateAround(const Translation2D &point,
                                const Rotation2D &rot) const {
    return {m_translation.rotateAround(point, rot), m_rotation.rotateBy(rot)};
  }

  /**
   * Obtain a new Pose2D from a (constant curvature) velocity.
   *
   * See https://file.tavsys.net/control/controls-engineering-in-frc.pdf section
   * 10.2 "Pose exponential" for a derivation.
   *
   * The twist is a change in pose in the robot's coordinate frame since the
   * previous pose update. When the user runs exp() on the previous known
   * field-relative pose with the argument being the twist, the user will
   * receive the new field-relative pose.
   *
   * "Exp" represents the pose exponential, which is solving a differential
   * equation moving the pose forward in time.
   *
   * @param twist The change in pose in the robot's coordinate frame since the
   * previous pose update. For example, if a non-holonomic robot moves forward
   * 0.01 meters and changes angle by 0.5 degrees since the previous pose
   * update, the twist would be Twist2d{0.01_m, 0_m, 0.5_deg}.
   *
   * @return The new pose of the robot.
   */
  constexpr Pose2D exp(const Pose2D &twist) const;

  /**
   * Returns a Twist2d that maps this pose to the end pose. If c is the output
   * of a.Log(b), then a.Exp(c) would yield b.
   *
   * @param end The end pose for the transformation.
   *
   * @return The twist that maps this to end.
   */
  constexpr Pose2D log(const Pose2D &end) const;

private:
  Translation2D m_translation;
  Rotation2D m_rotation;
};

constexpr Pose2D Pose2D::operator-(const Pose2D &other) const {
  const auto pose = this->relativeTo(other);
  return Pose2D{pose.translation(), pose.rotation()};
}

constexpr Pose2D Pose2D::transformBy(const Pose2D &other) const {
  return {m_translation + (other.translation().rotateBy(m_rotation)),
          other.rotation().rotateBy(m_rotation)};
}

constexpr Pose2D Pose2D::relativeTo(const Pose2D &other) const {
  Translation2D translation = (this->translation() - other.translation()).rotateBy(-other.rotation());
  Rotation2D rotation = this->rotation().relativeTo(other.rotation());
  return {translation, rotation};
}

constexpr Pose2D Pose2D::exp(const Pose2D &twist) const {
  const auto dx = twist.x();
  const auto dy = twist.y();
  const auto dtheta = twist.rotation().radians().value();

  const auto sinTheta = twist.rotation().sin();
  const auto cosTheta = twist.rotation().cos();

  double s, c;
  if (gcem::abs(dtheta) < 1E-9) {
    s = 1.0 - 1.0 / 6.0 * dtheta * dtheta;
    c = 0.5 * dtheta;
  } else {
    s = sinTheta / dtheta;
    c = (1 - cosTheta) / dtheta;
  }

  const Pose2D transform{Translation2D{dx * s - dy * c, dx * c + dy * s},
                              Rotation2D{cosTheta, sinTheta}};

  return *this + transform;
}

constexpr Pose2D Pose2D::log(const Pose2D &end) const {
  const auto transform = end.relativeTo(*this);
  const auto dtheta = transform.rotation().radians().value();
  const auto halfDtheta = dtheta / 2.0;

  const auto cosMinusOne = transform.rotation().cos() - 1;

  double halfThetaByTanOfHalfDtheta;

  if (gcem::abs(cosMinusOne) < 1E-9) {
    halfThetaByTanOfHalfDtheta = 1.0 - 1.0 / 12.0 * dtheta * dtheta;
  } else {
    halfThetaByTanOfHalfDtheta =
        -(halfDtheta * transform.rotation().sin()) / cosMinusOne;
  }

  const Translation2D translationPart =
      transform.translation().rotateBy(
          {halfThetaByTanOfHalfDtheta, -halfDtheta}) *
      gcem::hypot(halfThetaByTanOfHalfDtheta, halfDtheta);

  return {translationPart.x(), translationPart.y(), units::radian_t{dtheta}};
}

} // namespace libmavnetics

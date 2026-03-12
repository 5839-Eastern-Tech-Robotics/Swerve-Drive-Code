// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include "libmavnetics/utils/pose2d.hpp"
#include "libmavnetics/utils/rotation2d.hpp"
#include "units/angle.h"
#include "units/angular_velocity.h"
#include "units/time.h"
#include "units/velocity.h"

namespace libmavnetics {

struct ChassisSpeeds {
  units::meters_per_second_t vx = 0_mps;
  units::meters_per_second_t vy = 0_mps;
  units::radians_per_second_t omega = 0_rad_per_s;

  constexpr Pose2D getDistanceFromTimestep(units::second_t dt) const {
    return Pose2D{vx * dt, vy * dt, omega * dt};
  }

  /**
   * Discretizes a continuous-time chassis speed.
   *
   * This function converts a continuous-time chassis speed into a discrete-time
   * one such that when the discrete-time chassis speed is applied for one
   * timestep, the robot moves as if the velocity components are independent
   * (i.e., the robot moves v_x * dt along the x-axis, v_y * dt along the
   * y-axis, and omega * dt around the z-axis).
   *
   * This is useful for compensating for translational skew when translating and
   * rotating a holonomic (swerve or mecanum) drivetrain. However, scaling down
   * the ChassisSpeeds after discretizing (e.g., when desaturating swerve module
   * speeds) rotates the direction of net motion in the opposite direction of
   * rotational velocity, introducing a different translational skew which is
   * not accounted for by discretization.
   *
   * @param vx Forward velocity.
   * @param vy Sideways velocity.
   * @param omega Angular velocity.
   * @param dt The duration of the timestep the speeds should be applied for.
   *
   * @return Discretized ChassisSpeeds.
   */
  static constexpr ChassisSpeeds discretize(units::meters_per_second_t vx,
                                            units::meters_per_second_t vy,
                                            units::radians_per_second_t omega,
                                            units::second_t dt) {
    // Construct the desired pose after a timestep, relative to the current
    // pose. The desired pose has decoupled translation and rotation.
    Pose2D desiredDeltaPose{vx * dt, vy * dt, omega * dt};

    // Find the chassis translation/rotation deltas in the robot frame that move
    // the robot from its current pose to the desired pose
    auto twist = Pose2D{}.log(desiredDeltaPose);

    // Turn the chassis translation/rotation deltas into average velocities
    return {twist.x() / dt, twist.y() / dt, twist.rotation().degrees() / dt};
  }

  /**
   * Discretizes a continuous-time chassis speed.
   *
   * This function converts a continuous-time chassis speed into a discrete-time
   * one such that when the discrete-time chassis speed is applied for one
   * timestep, the robot moves as if the velocity components are independent
   * (i.e., the robot moves v_x * dt along the x-axis, v_y * dt along the
   * y-axis, and omega * dt around the z-axis).
   *
   * This is useful for compensating for translational skew when translating and
   * rotating a holonomic (swerve or mecanum) drivetrain. However, scaling down
   * the ChassisSpeeds after discretizing (e.g., when desaturating swerve module
   * speeds) rotates the direction of net motion in the opposite direction of
   * rotational velocity, introducing a different translational skew which is
   * not accounted for by discretization.
   *
   * @param continuousSpeeds The continuous speeds.
   * @param dt The duration of the timestep the speeds should be applied for.
   *
   * @return Discretized ChassisSpeeds.
   */
  static constexpr ChassisSpeeds discretize(
      const ChassisSpeeds& continuousSpeeds, units::second_t dt) {
    return discretize(continuousSpeeds.vx, continuousSpeeds.vy,
                      continuousSpeeds.omega, dt);
  }

  /**
   * Converts a user provided field-relative set of speeds into a robot-relative
   * ChassisSpeeds object.
   *
   * @param vx The component of speed in the x direction relative to the field.
   * Positive x is away from your alliance wall.
   * @param vy The component of speed in the y direction relative to the field.
   * Positive y is to your left when standing behind the alliance wall.
   * @param omega The angular rate of the robot.
   * @param robotAngle The angle of the robot as measured by a gyroscope. The
   * robot's angle is considered to be zero when it is facing directly away from
   * your alliance station wall. Remember that this should be CCW positive.
   *
   * @return ChassisSpeeds object representing the speeds in the robot's frame
   * of reference.
   */
  static constexpr ChassisSpeeds fromFieldRelativeSpeeds(
      units::meters_per_second_t vx, units::meters_per_second_t vy,
      units::radians_per_second_t omega, const Rotation2D& robotAngle) {
    // CW rotation into chassis frame
    auto rotated =
        Translation2D{units::meter_t{vx.value()}, units::meter_t{vy.value()}}
            .rotateBy(-robotAngle);
    return {units::meters_per_second_t{rotated.x().value()},
            units::meters_per_second_t{rotated.y().value()}, omega};
  }

  /**
   * Converts a user provided field-relative ChassisSpeeds object into a
   * robot-relative ChassisSpeeds object.
   *
   * @param fieldRelativeSpeeds The ChassisSpeeds object representing the speeds
   *    in the field frame of reference. Positive x is away from your alliance
   *    wall. Positive y is to your left when standing behind the alliance wall.
   * @param robotAngle The angle of the robot as measured by a gyroscope. The
   *    robot's angle is considered to be zero when it is facing directly away
   *    from your alliance station wall. Remember that this should be CCW
   *    positive.
   * @return ChassisSpeeds object representing the speeds in the robot's frame
   *    of reference.
   */
  static constexpr ChassisSpeeds fromFieldRelativeSpeeds(
      const ChassisSpeeds& fieldRelativeSpeeds, const Rotation2D& robotAngle) {
    return fromFieldRelativeSpeeds(fieldRelativeSpeeds.vx,
                                   fieldRelativeSpeeds.vy,
                                   fieldRelativeSpeeds.omega, robotAngle);
  }

  /**
   * Converts a user provided robot-relative set of speeds into a field-relative
   * ChassisSpeeds object.
   *
   * @param vx The component of speed in the x direction relative to the robot.
   * Positive x is towards the robot's front.
   * @param vy The component of speed in the y direction relative to the robot.
   * Positive y is towards the robot's left.
   * @param omega The angular rate of the robot.
   * @param robotAngle The angle of the robot as measured by a gyroscope. The
   * robot's angle is considered to be zero when it is facing directly away from
   * your alliance station wall. Remember that this should be CCW positive.
   *
   * @return ChassisSpeeds object representing the speeds in the field's frame
   * of reference.
   */
  static constexpr ChassisSpeeds fromRobotRelativeSpeeds(
      units::meters_per_second_t vx, units::meters_per_second_t vy,
      units::radians_per_second_t omega, const Rotation2D& robotAngle) {
    // CCW rotation out of chassis frame
    auto rotated =
        Translation2D{units::meter_t{vx.value()}, units::meter_t{vy.value()}}
            .rotateBy(robotAngle);
    return {units::meters_per_second_t{rotated.x().value()},
            units::meters_per_second_t{rotated.y().value()}, omega};
  }

  /**
   * Converts a user provided robot-relative ChassisSpeeds object into a
   * field-relative ChassisSpeeds object.
   *
   * @param robotRelativeSpeeds The ChassisSpeeds object representing the speeds
   *    in the robot frame of reference. Positive x is the towards robot's
   * front. Positive y is towards the robot's left.
   * @param robotAngle The angle of the robot as measured by a gyroscope. The
   *    robot's angle is considered to be zero when it is facing directly away
   *    from your alliance station wall. Remember that this should be CCW
   *    positive.
   * @return ChassisSpeeds object representing the speeds in the field's frame
   *    of reference.
   */
  static constexpr ChassisSpeeds fromRobotRelativeSpeeds(
      const ChassisSpeeds& robotRelativeSpeeds, const Rotation2D& robotAngle) {
    return fromRobotRelativeSpeeds(robotRelativeSpeeds.vx,
                                   robotRelativeSpeeds.vy,
                                   robotRelativeSpeeds.omega, robotAngle);
  }

  /**
   * Adds two ChassisSpeeds and returns the sum.
   *
   * <p>For example, ChassisSpeeds{1.0, 0.5, 1.5} + ChassisSpeeds{2.0, 1.5, 0.5}
   * = ChassisSpeeds{3.0, 2.0, 2.0}
   *
   * @param other The ChassisSpeeds to add.
   *
   * @return The sum of the ChassisSpeeds.
   */
  constexpr ChassisSpeeds operator+(const ChassisSpeeds& other) const {
    return {vx + other.vx, vy + other.vy, omega + other.omega};
  }

  /**
   * Subtracts the other ChassisSpeeds from the current ChassisSpeeds and
   * returns the difference.
   *
   * <p>For example, ChassisSpeeds{5.0, 4.0, 2.0} - ChassisSpeeds{1.0, 2.0, 1.0}
   * = ChassisSpeeds{4.0, 2.0, 1.0}
   *
   * @param other The ChassisSpeeds to subtract.
   *
   * @return The difference between the two ChassisSpeeds.
   */
  constexpr ChassisSpeeds operator-(const ChassisSpeeds& other) const {
    return *this + -other;
  }

  /**
   * Returns the inverse of the current ChassisSpeeds.
   * This is equivalent to negating all components of the ChassisSpeeds.
   *
   * @return The inverse of the current ChassisSpeeds.
   */
  constexpr ChassisSpeeds operator-() const { return {-vx, -vy, -omega}; }

  /**
   * Multiplies the ChassisSpeeds by a scalar and returns the new ChassisSpeeds.
   *
   * <p>For example, ChassisSpeeds{2.0, 2.5, 1.0} * 2
   * = ChassisSpeeds{4.0, 5.0, 1.0}
   *
   * @param scalar The scalar to multiply by.
   *
   * @return The scaled ChassisSpeeds.
   */
  constexpr ChassisSpeeds operator*(double scalar) const {
    return {scalar * vx, scalar * vy, scalar * omega};
  }

  /**
   * Divides the ChassisSpeeds by a scalar and returns the new ChassisSpeeds.
   *
   * <p>For example, ChassisSpeeds{2.0, 2.5, 1.0} / 2
   * = ChassisSpeeds{1.0, 1.25, 0.5}
   *
   * @param scalar The scalar to divide by.
   *
   * @return The scaled ChassisSpeeds.
   */
  constexpr ChassisSpeeds operator/(double scalar) const {
    return operator*(1.0 / scalar);
  }

  /**
   * Compares the ChassisSpeeds with another ChassisSpeed.
   *
   * @param other The other ChassisSpeeds.
   *
   * @return The result of the comparison. Is true if they are the same.
   */
  constexpr bool operator==(const ChassisSpeeds& other) const = default;
};
}  // namespace libmavnetics

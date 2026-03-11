#pragma once

#include <array>
#include <cstddef>

#include "robot/drive/odometry.hpp"
#include "robot/drive/swerveModule.hpp"
#include "robot/utils/pose2d.hpp"
#include "swerveModule.hpp"
#include "units/angle.h"
#include "units/angular_velocity.h"
#include "units/time.h"
#include "units/velocity.h"

namespace libmavnetics {

// WheelSpeeds std::array<SwerveModuleState, numModules>
// WheelPositions std::array<SwerveModulePosition, numModules>

// allwpilib/wpimath/src/main/native/include/frc/kinematics

template <size_t numModules> class SwerveDriveKinematics {
 public:
  /**
   * Performs forward kinematics to return the resulting chassis speed from the
   * wheel speeds. This method is often used for odometry -- determining the
   * robot's position on the field using data from the real-world speed of each
   * wheel on the robot.
   *
   * @param wheelSpeeds The speeds of the wheels.
   * @return The chassis speed.
   */
  virtual ChassisSpeeds ToChassisSpeeds(
      const std::array<SwerveModulePosition, numModules>& wheelSpeeds) const = 0;

  /**
   * Performs inverse kinematics to return the wheel speeds from a desired
   * chassis velocity. This method is often used to convert joystick values into
   * wheel speeds.
   *
   * @param chassisSpeeds The desired chassis speed.
   * @return The wheel speeds.
   */
  virtual std::array<SwerveModulePosition, numModules> ToWheelSpeeds(
      const ChassisSpeeds& chassisSpeeds) const = 0;

  /**
   * Performs forward kinematics to return the resulting Pose2d from the given
   * change in wheel positions. This method is often used for odometry --
   * determining the robot's position on the field using changes in the distance
   * driven by each wheel on the robot.
   *
   * @param start The starting distances driven by the wheels.
   * @param end The ending distances driven by the wheels.
   *
   * @return The resulting Pose2d in the robot's movement.
   */
  virtual Pose2d ToPose2d(const std::array<SwerveModulePosition, numModules>& start,
                            const std::array<SwerveModulePosition, numModules>& end) const = 0;

  /**
   * Performs interpolation between two values.
   *
   * @param start The value to start at.
   * @param end The value to end at.
   * @param t How far between the two values to interpolate. This should be
   * bounded to [0, 1].
   * @return The interpolated value.
   */
  virtual std::array<SwerveModulePosition, numModules> Interpolate(const std::array<SwerveModulePosition, numModules>& start,
                                     const std::array<SwerveModulePosition, numModules>& end,
                                     double t) const = 0;
};

class SwerveDrive {
  SwerveDrive(std::array<SwerveModule, 4> modules,
              SwerveDriveKinematics<4> kinematics, Odometry odometry);

  void update();

  void drive(units::meters_per_second_t xSpeed,
             units::meters_per_second_t ySpeed,
             units::radians_per_second rotSpeed, bool fieldRelative,
             units::second_t period);

  void reset();
  void setModuleStates(std::array<SwerveModuleState, 4> desiredStates);

  units::degree_t getHeading();
  void tareHeading();
  units::degrees_per_second_t getTurnVelocity();

  Pose2D getPose();
  void resetOdometry(Pose2D pose);

private:
  SwerveModule m_frontLeft;
  SwerveModule m_backLeft;
  SwerveModule m_frontRight;
  SwerveModule m_backRight;

  SwerveDriveKinematics<4> m_kinematics{};

  Odometry m_odometry;
};

} // namespace libmavnetics
// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <wpi/SymbolExports.h>

#include "frc/geometry/Twist2d.h"
#include "frc/kinematics/ChassisSpeeds.h"

//namespace frc {
/**
 * Helper class that converts a chassis velocity (dx, dy, and dtheta components)
 * into individual wheel speeds. Robot code should not use this directly-
 * Instead, use the particular type for your drivetrain (e.g.,
 * DifferentialDriveKinematics).
 *
 * Inverse kinematics converts a desired chassis speed into wheel speeds whereas
 * forward kinematics converts wheel speeds into chassis speed.
 */

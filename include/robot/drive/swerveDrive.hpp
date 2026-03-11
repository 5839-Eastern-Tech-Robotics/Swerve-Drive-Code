#pragma once

#include <array>
#include <cstddef>

#include "robot/drive/odometry.hpp"
#include "robot/drive/swerveModule.hpp"
#include "robot/utils/chassisSpeeds.hpp"
#include "robot/utils/pose2d.hpp"
#include "robot/utils/rotation2d.hpp"
#include "units/angle.h"
#include "units/angular_velocity.h"
#include "units/time.h"
#include "units/velocity.h"

namespace libmavnetics {

// allwpilib/wpimath/src/main/native/include/frc/kinematics

template <size_t numModules> class SwerveDriveKinematics {
public:
  using WheelSpeeds = std::array<SwerveModuleState, numModules>;
  using WheelPositions = std::array<SwerveModulePosition, numModules>;

  /**
   * Performs forward kinematics to return the resulting chassis speed from the
   * wheel speeds. This method is often used for odometry -- determining the
   * robot's position on the field using data from the real-world speed of each
   * wheel on the robot.
   *
   * @param wheelSpeeds The speeds of the wheels.
   * @return The chassis speed.
   */
  ChassisSpeeds toChassisSpeeds(const WheelSpeeds &wheelSpeeds) const;

  /**
   * Performs inverse kinematics to return the wheel speeds from a desired
   * chassis velocity. This method is often used to convert joystick values into
   * wheel speeds.
   *
   * @param chassisSpeeds The desired chassis speed.
   * @return The wheel speeds.
   */
  WheelSpeeds toWheelSpeeds(const ChassisSpeeds &chassisSpeeds) const;

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
  Pose2D toPose2D(const WheelPositions &start, const WheelPositions &end) const;

  /**
   * Performs interpolation between two values.
   *
   * @param start The value to start at.
   * @param end The value to end at.
   * @param t How far between the two values to interpolate. This should be
   * bounded to [0, 1].
   * @return The interpolated value.
   */
  WheelPositions interpolate(const WheelPositions &start,
                             const WheelPositions &end, double t) const;

  static void
  desaturateWheelSpeeds(WheelSpeeds *moduleStates,
                        units::meters_per_second_t attainableMaxSpeed);
};

class SwerveDrive {
  /**
   * Initializes the swerve drive
   * @param modules the swerve drive modules in the order [fl, fr, bl, br]
   */
  SwerveDrive(std::array<SwerveModule, 4> modules,
              SwerveDriveKinematics<4> kinematics, Odometry odometry);

  void calibrate();
  void update();

  void drive(units::meters_per_second_t xSpeed,
             units::meters_per_second_t ySpeed,
             units::radians_per_second_t rotSpeed, bool fieldRelative = true,
             units::second_t period = 20_ms);

  void reset();
  void setModuleStates(std::array<SwerveModuleState, 4> desiredStates);

  Rotation2D getHeading();
  Pose2D getPose();

  void setPose(units::meter_t x, units::meter_t y, units::degree_t theta);
  void setPose(units::meter_t x, units::meter_t y, Rotation2D theta);
  void setPose(Translation2D position, units::degree_t theta);
  void setPose(Translation2D position, Rotation2D theta);
  void setPose(Pose2D pose);

private:
  SwerveModule m_frontLeft;
  SwerveModule m_backLeft;
  SwerveModule m_frontRight;
  SwerveModule m_backRight;

  SwerveDriveKinematics<4> m_kinematics{};

  Odometry m_odometry;
};

} // namespace libmavnetics

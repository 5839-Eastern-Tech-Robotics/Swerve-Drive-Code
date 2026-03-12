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

struct empty_array_t {};
constexpr empty_array_t empty_array;

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


  WheelSpeeds toWheelSpeeds(const ChassisSpeeds &chassisSpeeds) const override {
      return toSwerveModuleStates(chassisSpeeds);
  }

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
  
  /**
   * Constructs a swerve drive kinematics object. This takes in a variable
   * number of module locations as Translation2ds. The order in which you pass
   * in the module locations is the same order that you will receive the module
   * states when performing inverse kinematics. It is also expected that you
   * pass in the module states in the same order when calling the forward
   * kinematics methods.
   *
   * @param moduleTranslations The locations of the modules relative to the
   *                           physical center of the robot.
   */
  template <std::convertible_to<Translation2D>... ModuleTranslations>
    requires(sizeof...(ModuleTranslations) == numModules)
  explicit SwerveDriveKinematics(ModuleTranslations&&... moduleTranslations)
      : m_modules{moduleTranslations...}, m_moduleHeadings(empty_array) {
    for (size_t i = 0; i < numModules; i++) {
      // clang-format off
      m_inverseKinematics.template block<2, 3>(i * 2, 0) <<
        1, 0, (-m_modules[i].Y()).value(),
        0, 1, (+m_modules[i].X()).value();
      // clang-format on
    }

    m_forwardKinematics = m_inverseKinematics.householderQr();

    // wpi::math::MathSharedStore::ReportUsage(
    //     wpi::math::MathUsageId::kKinematics_SwerveDrive, 1);
  }

  explicit SwerveDriveKinematics(
      const std::array<Translation2D, numModules>& modules)
      : m_modules{modules}, m_moduleHeadings(empty_array) {
    for (size_t i = 0; i < numModules; i++) {
      // clang-format off
      m_inverseKinematics.template block<2, 3>(i * 2, 0) <<
        1, 0, (-m_modules[i].Y()).value(),
        0, 1, (+m_modules[i].X()).value();
      // clang-format on
    }

    m_forwardKinematics = m_inverseKinematics.householderQr();

    // wpi::math::MathSharedStore::ReportUsage(
    //     wpi::math::MathUsageId::kKinematics_SwerveDrive, 1);
  }

  SwerveDriveKinematics(const SwerveDriveKinematics&) = default;

  /**
   * Reset the internal swerve module headings.
   * @param moduleHeadings The swerve module headings. The order of the module
   * headings should be same as passed into the constructor of this class.
   */
  template <std::convertible_to<Rotation2D>... ModuleHeadings>
    requires(sizeof...(ModuleHeadings) == numModules)
  void ResetHeadings(ModuleHeadings&&... moduleHeadings) {
    return this->ResetHeadings(
        std::array<Rotation2D, numModules>{moduleHeadings...});
  }


  /**
   * Performs inverse kinematics to return the module states from a desired
   * chassis velocity. This method is often used to convert joystick values into
   * module speeds and angles.
   *
   * This function also supports variable centers of rotation. During normal
   * operations, the center of rotation is usually the same as the physical
   * center of the robot; therefore, the argument is defaulted to that use case.
   * However, if you wish to change the center of rotation for evasive
   * maneuvers, vision alignment, or for any other use case, you can do so.
   *
   * In the case that the desired chassis speeds are zero (i.e. the robot will
   * be stationary), the previously calculated module angle will be maintained.
   *
   * @param chassisSpeeds The desired chassis speed.
   * @param centerOfRotation The center of rotation. For example, if you set the
   * center of rotation at one corner of the robot and provide a chassis speed
   * that only has a dtheta component, the robot will rotate around that corner.
   *
   * @return An array containing the module states. Use caution because these
   * module states are not normalized. Sometimes, a user input may cause one of
   * the module speeds to go above the attainable max velocity. Use the
   * DesaturateWheelSpeeds(wpi::array<SwerveModuleState, NumModules>*,
   * units::meters_per_second_t) function to rectify this issue. In addition,
   * you can leverage the power of C++17 to directly assign the module states to
   * variables:
   *
   * @code{.cpp}
   * auto [fl, fr, bl, br] = kinematics.ToSwerveModuleStates(chassisSpeeds);
   * @endcode
   */
  WheelSpeeds toSwerveModuleStates(
      const ChassisSpeeds& chassisSpeeds,
      const Translation2D& centerOfRotation = Translation2D{}) const {
    WheelSpeeds moduleStates(empty_array);

    if (chassisSpeeds.vx == 0_mps && chassisSpeeds.vy == 0_mps &&
        chassisSpeeds.omega == 0_rad_per_s) {
      for (size_t i = 0; i < numModules; i++) {
        moduleStates[i] = {0_mps, m_moduleHeadings[i]};
      }

      return moduleStates;
    }

    // We have a new center of rotation. We need to compute the matrix again.
    if (centerOfRotation != m_previousCoR) {
      for (size_t i = 0; i < numModules; i++) {
        // clang-format off
      m_inverseKinematics.template block<2, 3>(i * 2, 0) =
        Matrixd<2, 3>{
          {1, 0, (-m_modules[i].Y() + centerOfRotation.Y()).value()},
          {0, 1, (+m_modules[i].X() - centerOfRotation.X()).value()}};
        // clang-format on
      }
      m_previousCoR = centerOfRotation;
    }

    // TODO: Eigen???
    Eigen::Vector3d chassisSpeedsVector{chassisSpeeds.vx.value(),
                                        chassisSpeeds.vy.value(),
                                        chassisSpeeds.omega.value()};

    Matrixd<numModules * 2, 1> moduleStateMatrix =
        m_inverseKinematics * chassisSpeedsVector;

    for (size_t i = 0; i < numModules; i++) {
      units::meters_per_second_t x{moduleStateMatrix(i * 2, 0)};
      units::meters_per_second_t y{moduleStateMatrix(i * 2 + 1, 0)};

      auto speed = units::math::hypot(x, y);
      auto rotation = speed > 1e-6_mps ? Rotation2D{x.value(), y.value()}
                                       : m_moduleHeadings[i];

      moduleStates[i] = {speed, rotation};
      m_moduleHeadings[i] = rotation;
    }

    return moduleStates;
  }

 private:
  std::array<Translation2D, numModules> m_modules;
  mutable Matrixd<numModules * 2, 3> m_inverseKinematics;
  Eigen::HouseholderQR<Matrixd<numModules * 2, 3>> m_forwardKinematics;
  mutable std::array<Rotation2D, numModules> m_moduleHeadings;

  mutable Translation2d m_previousCoR;
};

template <typename ModuleTranslation, typename... ModuleTranslations>
SwerveDriveKinematics(ModuleTranslation, ModuleTranslations...)
    -> SwerveDriveKinematics<1 + sizeof...(ModuleTranslations)>;
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

#pragma once

#include <array>
#include <cstddef>

#include "Eigen/Core"
#include "Eigen/QR"
#include "libmavnetics/drive/odometry.hpp"
#include "libmavnetics/drive/swerveModule.hpp"
//#include "libmavnetics/utils/trajectory.hpp"
#include "libmavnetics/utils/chassisSpeeds.hpp"
#include "libmavnetics/utils/pose2d.hpp"
#include "libmavnetics/utils/rotation2d.hpp"
#include "libmavnetics/utils/translation2d.hpp"
#include "libmavnetics/utils/twist2d.hpp"
#include "libmavnetics/utils/pid.hpp"
#include "units/angle.h"
#include "units/angular_velocity.h"
#include "units/length.h"
#include "units/time.h"
#include "units/velocity.h"

namespace libmavnetics {

template <int Size> using Vectord = Eigen::Vector<double, Size>;

template <int Rows, int Cols,
          int Options = Eigen::AutoAlign |
                        ((Rows == 1 && Cols != 1) ? Eigen::RowMajor
                         : (Cols == 1 && Rows != 1)
                             ? Eigen::ColMajor
                             : EIGEN_DEFAULT_MATRIX_STORAGE_ORDER_OPTION),
          int MaxRows = Rows, int MaxCols = Cols>
using Matrixd = Eigen::Matrix<double, Rows, Cols, Options, MaxRows, MaxCols>;

template <size_t numModules> class SwerveDriveKinematics {
public:
  using WheelSpeeds = std::array<SwerveModuleState, numModules>;
  using WheelPositions = std::array<SwerveModulePosition, numModules>;

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
  explicit SwerveDriveKinematics(ModuleTranslations &&...moduleTranslations)
      : m_modules{moduleTranslations...}, m_moduleHeadings({}) {
    for (size_t i = 0; i < numModules; i++) {
      // clang-format off
      m_inverseKinematics.template block<2, 3>(i * 2, 0) <<
        1, 0, (-m_modules[i].y()).value(),
        0, 1, (+m_modules[i].x()).value();
      // clang-format on
    }

    m_forwardKinematics = m_inverseKinematics.householderQr();
  }

  explicit SwerveDriveKinematics(
      const std::array<Translation2D, numModules> &modules)
      : m_modules{modules}, m_moduleHeadings({}) {
    for (size_t i = 0; i < numModules; i++) {
      // clang-format off
      m_inverseKinematics.template block<2, 3>(i * 2, 0) <<
        1, 0, (-m_modules[i].y()).value(),
        0, 1, (+m_modules[i].x()).value();
      // clang-format on
    }

    m_forwardKinematics = m_inverseKinematics.householderQr();
  }

  SwerveDriveKinematics(const SwerveDriveKinematics &) = default;

  /**
   * Performs inverse kinematics to return the wheel speeds from a desired
   * chassis velocity. This method is often used to convert joystick values into
   * wheel speeds.
   *
   * @param chassisSpeeds The desired chassis speed.
   * @return The wheel speeds.
   */
  WheelSpeeds toWheelSpeeds(const ChassisSpeeds &chassisSpeeds) const {
    return toSwerveModuleStates(chassisSpeeds);
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
      const ChassisSpeeds &chassisSpeeds,
      const Translation2D &centerOfRotation = Translation2D{}) const {
    WheelSpeeds moduleStates({});

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
          {1, 0, (-m_modules[i].y() + centerOfRotation.y()).value()},
          {0, 1, (+m_modules[i].x() - centerOfRotation.x()).value()}};
        // clang-format on
      }
      m_previousCoR = centerOfRotation;
    }

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
   * Performs forward kinematics to return the resulting Twist2D from the given
   * change in wheel positions. This method is often used for odometry --
   * determining the robot's position on the field using changes in the distance
   * driven by each wheel on the robot.
   *
   * @param start The starting distances driven by the wheels.
   * @param end The ending distances driven by the wheels.
   *
   * @return The resulting Twist2D in the robot's movement.
   */
  Twist2D toTwist2D(const WheelPositions &start,
                    const WheelPositions &end) const;

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
                        units::meters_per_second_t attainableMaxSpeed) {
    auto &states = *moduleStates;
    auto realMaxSpeed =
        units::math::abs(std::max_element(states.begin(), states.end(),
                                          [](const auto &a, const auto &b) {
                                            return units::math::abs(a.speed) <
                                                   units::math::abs(b.speed);
                                          })
                             ->speed);

    if (realMaxSpeed > attainableMaxSpeed) {
      for (auto &module : states) {
        module.speed = module.speed / realMaxSpeed * attainableMaxSpeed;
      }
    }
  }
  /**
   * Reset the internal swerve module headings.
   * @param moduleHeadings The swerve module headings. The order of the module
   * headings should be same as passed into the constructor of this class.
   */
  template <std::convertible_to<Rotation2D>... ModuleHeadings>
    requires(sizeof...(ModuleHeadings) == numModules)
  void resetHeadings(ModuleHeadings &&...moduleHeadings) {
    return this->resetHeadings(
        std::array<Rotation2D, numModules>{moduleHeadings...});
  }

private:
  std::array<Translation2D, numModules> m_modules;
  mutable Matrixd<numModules * 2, 3> m_inverseKinematics;
  Eigen::HouseholderQR<Matrixd<numModules * 2, 3>> m_forwardKinematics;
  mutable std::array<Rotation2D, numModules> m_moduleHeadings;

  mutable Translation2D m_previousCoR;
};

template <typename ModuleTranslation, typename... ModuleTranslations>
SwerveDriveKinematics(ModuleTranslation, ModuleTranslations...)
    -> SwerveDriveKinematics<1 + sizeof...(ModuleTranslations)>;

class SwerveDrive {
public:
  /**
   * Initializes the swerve drive
   * @param modules the swerve drive modules in the order [fl, fr, bl, br]
   */
  SwerveDrive(std::array<SwerveModule, 4> modules, units::meter_t track_width,
              units::meter_t wheel_base, Odometry *odometry);

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

  void driveToPose(Pose2D trajectoryPose, units::meters_per_second_t desiredLinearVelocity, const Rotation2D& desiredHeading, bool async);
  bool isFinishedMovement();
  void waitUntilDone();
  void waitUtilDistance(units::meter_t dist);
  //void driveTrajectory(Trajectory, bool async, ...);

private:
  SwerveModule m_frontLeft;
  SwerveModule m_backLeft;
  SwerveModule m_frontRight;
  SwerveModule m_backRight;

  units::meter_t track_width;
  units::meter_t wheel_base;

  SwerveDriveKinematics<4> m_kinematics;
  Odometry *m_odometry;
  
  // from allwpilib/wpimath/src/main/native/include/frc/controller/HolonomicDriveController.h
  
  Pose2D m_poseError;
  Rotation2D m_rotationError;
  Pose2D m_poseTolerance;
  bool m_enabled = true;

  PID m_xController;
  PID m_yController;
  PID m_thetaController;

  bool m_firstRun = true;
};

} // namespace libmavnetics

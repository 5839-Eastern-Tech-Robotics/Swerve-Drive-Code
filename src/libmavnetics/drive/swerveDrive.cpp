#include "libmavnetics/drive/swerveDrive.hpp"

#include <algorithm>
#include <array>

#include "libmavnetics/drive/odometry.hpp"
#include "libmavnetics/drive/swerveModule.hpp"
#include "libmavnetics/utils/rotation2d.hpp"

namespace libmavnetics {

SwerveDrive::SwerveDrive(std::array<SwerveModule, 4> modules,
                         units::meter_t track_width, units::meter_t wheel_base,
                         Odometry *odometry, PID *xPID, PID *yPID,
                         PID *thetaPID, PID *rotPID)
    : m_frontLeft(modules[0]), m_backLeft(modules[2]), m_frontRight(modules[1]),
      m_backRight(modules[3]), m_odometry(odometry), track_width(track_width),
      wheel_base(wheel_base), m_xController(xPID), m_yController(yPID),
      m_thetaController(thetaPID), m_rotController(rotPID),
      m_kinematics(Translation2D{wheel_base / 2.0, track_width / 2.0},
                   Translation2D{wheel_base / 2.0, -track_width / 2.0},
                   Translation2D{-wheel_base / 2.0, track_width / 2.0},
                   Translation2D{-wheel_base / 2.0, -track_width / 2.0}) {
  m_frontLeft.print = true;
}

void SwerveDrive::calibrate() { m_odometry->calibrate(); }

void SwerveDrive::update() {
  if (!m_odometry->isTaskRunning())
    m_odometry->update();
}

void SwerveDrive::drive(units::meters_per_second_t xSpeed,
                        units::meters_per_second_t ySpeed,
                        units::radians_per_second_t rotSpeed,
                        bool fieldRelative, units::second_t period) {
  ChassisSpeeds measuredSpeed = (fieldRelative ? m_odometry->getGlobalSpeed()
                                               : m_odometry->getLocalSpeed());

  // rotSpeed = units::radians_per_second_t{m_rotController->calculate(
  //     measuredSpeed.omega.value(), rotSpeed.value())};
  //
  // std::cout << "[CHASSIS_STR] " << measuredSpeed.omega.value() << " "
  //           << rotSpeed.value() << " " << m_rotController->getError() << " "
  //           << rotSpeed.value() << std::endl;

  auto states = m_kinematics.toWheelSpeeds(ChassisSpeeds::discretize(
      fieldRelative
          ? ChassisSpeeds::fromFieldRelativeSpeeds(
                xSpeed, ySpeed, rotSpeed, m_odometry->getPose().rotation())
          : ChassisSpeeds{xSpeed, ySpeed, rotSpeed},
      period));

  m_kinematics.desaturateWheelSpeeds(
      &states, std::max({m_frontLeft.maxSpeed(), m_frontRight.maxSpeed(),
                         m_backLeft.maxSpeed(), m_backRight.maxSpeed()}));

  auto [fl, fr, bl, br] = states;

  m_frontLeft.setDesiredState(fl);
  m_frontRight.setDesiredState(fr);
  m_backLeft.setDesiredState(bl);
  m_backRight.setDesiredState(br);
}

void SwerveDrive::setModuleStates(
    std::array<SwerveModuleState, 4> desiredStates) {

  m_kinematics.desaturateWheelSpeeds(
      &desiredStates,
      std::max({m_frontLeft.maxSpeed(), m_frontRight.maxSpeed(),
                m_backLeft.maxSpeed(), m_backRight.maxSpeed()}));

  auto [fl, fr, bl, br] = desiredStates;

  m_frontLeft.setDesiredState(fl);
  m_frontRight.setDesiredState(fr);
  m_backLeft.setDesiredState(bl);
  m_backRight.setDesiredState(br);
}

void SwerveDrive::reset() { m_odometry->resetPose(); }

Rotation2D SwerveDrive::getHeading() {
  return m_odometry->getPose().rotation();
}

Pose2D SwerveDrive::getPose() { return m_odometry->getPose(); }

void SwerveDrive::setPose(units::meter_t x, units::meter_t y,
                          units::degree_t theta) {
  m_odometry->setPose(x, y, theta);
}

void SwerveDrive::setPose(units::meter_t x, units::meter_t y,
                          Rotation2D theta) {
  m_odometry->setPose(x, y, theta);
}

void SwerveDrive::setPose(Translation2D position, units::degree_t theta) {
  m_odometry->setPose(position, theta);
}

void SwerveDrive::setPose(Translation2D position, Rotation2D theta) {
  m_odometry->setPose(position, theta);
}

void SwerveDrive::setPose(Pose2D pose) { m_odometry->setPose(pose); }

void SwerveDrive::driveToPose(Pose2D trajectoryPose,
                              units::meters_per_second_t desiredLinearVelocity,
                              const Rotation2D &desiredHeading, bool async) {
  auto currentPose = m_odometry->getPose();

  // Calculate feedforward velocities (field-relative)
  auto xFF = desiredLinearVelocity * trajectoryPose.rotation().cos();
  auto yFF = desiredLinearVelocity * trajectoryPose.rotation().sin();
  units::radians_per_second_t thetaFB{
      m_thetaController->calculate(currentPose.rotation().radians().value(),
                                   desiredHeading.radians().value())};

  // std::cout << "[CHASSIS_THETA] " << currentPose.rotation().radians().value()
  //           << " " << trajectoryPose.rotation().radians().value() << " "
  //           << m_thetaController->getError() << " " << thetaFB.value()
  //           << std::endl;

  m_poseError = trajectoryPose.relativeTo(currentPose);
  m_rotationError = desiredHeading - currentPose.rotation();

  std::array<SwerveModuleState, 4> states;
  if (!m_enabled) {
    states = m_kinematics.toWheelSpeeds(ChassisSpeeds::fromFieldRelativeSpeeds(
        xFF, yFF, thetaFB, currentPose.rotation()));
  } else {

    // Calculate feedback velocities (based on position error).
    auto xFeedback = units::meters_per_second_t{m_xController->calculate(
        currentPose.x().value(), trajectoryPose.x().value())};
    auto yFeedback = units::meters_per_second_t{m_yController->calculate(
        currentPose.y().value(), trajectoryPose.y().value())};

    // std::cout << "[CHASSIS_X] " << currentPose.x().value() << " "
    //           << trajectoryPose.x().value() << " " <<
    //           m_xController->getError()
    //           << " " << xFeedback.value() << std::endl;

    // std::cout << "[CHASSIS_Y] " << currentPose.y().value() << " "
    //           << trajectoryPose.y().value() << " " <<
    //           m_yController->getError()
    //           << " " << yFeedback.value() << std::endl;

    // Calculate states.
    states = m_kinematics.toWheelSpeeds(ChassisSpeeds::fromFieldRelativeSpeeds(
        xFF + xFeedback, yFF + yFeedback, thetaFB, currentPose.rotation()));
  }

  setModuleStates(states);
}

// bool isFinishedMovement();
// void waitUntilDone();
// void waitUtilDistance(units::meter_t dist);
// void driveTrajectory(Trajectory, bool async, ...);

} // namespace libmavnetics

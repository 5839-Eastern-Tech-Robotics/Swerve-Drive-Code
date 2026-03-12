#include "robot/drive/swerveDrive.hpp"
#include "robot/drive/odometry.hpp"
#include "robot/drive/swerveModule.hpp"
#include "robot/utils/chassisSpeeds.hpp"
#include "robot/utils/rotation2d.hpp"
#include <algorithm>
#include <array>

namespace libmavnetics {

SwerveDrive::SwerveDrive(std::array<SwerveModule, 4> modules,
                         units::meter_t track_width, units::meter_t wheel_base,
                         Odometry *odometry)
    : m_frontLeft(modules[0]), m_backLeft(modules[2]), m_frontRight(modules[1]),
      m_backRight(modules[3]), m_odometry(odometry), track_width(track_width),
      wheel_base(wheel_base),
      m_kinematics(Translation2D{wheel_base / 2.0, track_width / 2.0},
                   Translation2D{wheel_base / 2.0, -track_width / 2.0},
                   Translation2D{-wheel_base / 2.0, track_width / 2.0},
                   Translation2D{-wheel_base / 2.0, -track_width / 2.0}) {}

void SwerveDrive::calibrate() { m_odometry->calibrate(); }

void SwerveDrive::update() {
  if (!m_odometry->isTaskRunning())
    m_odometry->update();
}

void SwerveDrive::drive(units::meters_per_second_t xSpeed,
                        units::meters_per_second_t ySpeed,
                        units::radians_per_second_t rotSpeed,
                        bool fieldRelative, units::second_t period) {
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

Rotation2D SwerveDrive::getHeading() { return m_odometry->getPose().rotation(); }

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
} // namespace libmavnetics

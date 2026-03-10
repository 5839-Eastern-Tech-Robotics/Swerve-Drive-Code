#pragma once

#include <array>
#include <cstddef>

#include "robot/drive/odometry.hpp"
#include "robot/utils/pose2d.hpp"
#include "swerveModule.hpp"
#include "units/angle.h"
#include "units/angular_velocity.h"
#include "units/time.h"
#include "units/velocity.h"

namespace libmavnetics {

template <size_t numModules> class SwerveDriveKinematics {

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

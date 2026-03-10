#pragma once

#include "globals.hpp"
#include "pros/motors.hpp"
#include "robot/utils/pid.hpp"
#include "robot/utils/rotation2d.hpp"
#include "robot/utils/translation2d.hpp"
#include "robot/utils/util.hpp"
#include "units/length.h"
#include "units/velocity.h"
#include "units/angle.h"
#include "units/time.h"

namespace libmavnetics {

struct SwerveModuleState {
  units::meters_per_second_t speed = 0_mps;
  Rotation2D angle;

  constexpr bool operator==(const SwerveModuleState &other) const {
    return gcem::abs(speed - other.speed) < 1E-9_mps &&
           angle == other.angle;
  }

  constexpr void optimize(const Rotation2D& currentAngle) {
    auto delta = angle - currentAngle;
    if (gcem::abs(delta.degrees()) > 90_deg) {
      speed *= -1;
      angle = angle + Rotation2D{180_deg};
    }
  }

  constexpr void scaleCosine(const Rotation2D& currentAngle) {
    speed *= (angle - currentAngle).cos();
  }
};

struct SwerveModulePosition {
  units::meter_t distance = 0_m;
  Rotation2D angle;

  constexpr bool operator==(const SwerveModulePosition& other) const {
    return gcem::abs(distance - other.distance) < 1E-9_m &&
           angle == other.angle;
  }

  constexpr SwerveModulePosition interpolate(
      const SwerveModulePosition& endValue, double t) const {
    return {lerp(distance, endValue.distance, t),
            lerp(angle, endValue.angle, t)};
  }
};

class SwerveModule {
public:
  SwerveModule(pros::Motor driveMotor, PID drivePIDController, float driveGearRatio,
               pros::Motor turnMotor, PID turnPIDController, float turnGearRatio,
               units::meter_t driveWheelDiameter, Translation2D location);

  constexpr Translation2D getModuleLocation() { return location; };

  SwerveModuleState getState();
  SwerveModulePosition getPosition();

  void setDesiredState(SwerveModuleState& state);
  void reset();

private:
  pros::Motor driveMotor, turnMotor;
  PID drivePID, turnPID;
  float driveGearRatio, turnGearRatio;
  units::meter_t driveWheelDiameter;
  Translation2D location;
};

}

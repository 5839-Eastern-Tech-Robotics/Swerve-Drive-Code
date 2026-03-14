#pragma once

#include "libmavnetics/utils/pid.hpp"
#include "libmavnetics/utils/rotation2d.hpp"
#include "libmavnetics/utils/util.hpp"
#include "pros/abstract_motor.hpp"
#include "pros/motors.hpp"
#include "units/angle.h"
#include "units/angular_velocity.h"
#include "units/base.h"
#include "units/constants.h"
#include "units/length.h"
#include "units/time.h"
#include "units/velocity.h"

namespace libmavnetics {

struct SwerveModuleState {
  units::meters_per_second_t speed = 0_mps;
  Rotation2D angle;

  constexpr bool operator==(const SwerveModuleState &other) const {
    return gcem::abs(speed - other.speed) < 1E-9_mps && angle == other.angle;
  }

  constexpr void optimize(const Rotation2D &currentAngle) {
    auto delta = angle - currentAngle;
    if (gcem::abs(delta.degrees()) > 90_deg) {
      speed *= -1;
      angle = angle + Rotation2D{180_deg};
    }
  }

  constexpr void scaleCosine(const Rotation2D &currentAngle) {
    speed *= (angle - currentAngle).cos();
  }
};

struct SwerveModulePosition {
  units::meter_t distance = 0_m;
  Rotation2D angle;

  constexpr bool operator==(const SwerveModulePosition &other) const {
    return gcem::abs(distance - other.distance) < 1E-9_m &&
           angle == other.angle;
  }

  constexpr SwerveModulePosition
  interpolate(const SwerveModulePosition &endValue, double t) const {
    return {lerp(distance, endValue.distance, t),
            lerp(angle, endValue.angle, t)};
  }
};

class SwerveModule {
public:
  SwerveModule(pros::Motor driveMotor, PID drivePIDController,
               const double driveGearRatio, pros::Motor turnMotor,
               PID turnPIDController, const double turnGearRatio,
               const units::meter_t driveWheelDiameter);

  SwerveModuleState getState();
  SwerveModulePosition getPosition();

  units::meters_per_second_t maxSpeed();

  void setDesiredState(SwerveModuleState &state);
  void reset();

private:
  pros::Motor driveMotor, turnMotor;
  PID drivePID;
  PID turnPID;
  const double driveGearRatio, turnGearRatio;
  const units::meter_t driveWheelDiameter;
};

} // namespace libmavnetics

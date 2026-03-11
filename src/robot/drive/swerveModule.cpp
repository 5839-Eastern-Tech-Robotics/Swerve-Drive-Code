#include "robot/drive/swerveModule.hpp"
#include "globals.hpp"
#include "robot/utils/rotation2d.hpp"
#include "robot/utils/translation2d.hpp"
#include "units/angle.h"
#include "units/length.h"
#include "units/time.h"
#include <numbers>

namespace libmavnetics {

SwerveModule::SwerveModule(pros::Motor driveMotor, PID drivePIDController,
                           float driveGearRatio, pros::Motor turnMotor,
                           PID turnPIDController, float turnGearRatio,
                           units::meter_t driveWheelDiameter)
    : driveMotor(driveMotor), drivePID(drivePIDController),
      driveGearRatio(driveGearRatio), turnMotor(turnMotor),
      turnPID(turnPIDController), turnGearRatio(turnGearRatio),
      driveWheelDiameter(driveWheelDiameter) {
  turnPID.enableContinuousInput(-180, 180);
}

SwerveModuleState SwerveModule::getState() {
  return {driveMotor.get_actual_velocity() * driveGearRatio * std::numbers::pi *
              driveWheelDiameter / 60_s,
          turnMotor.get_position() * turnGearRatio * 1_deg};
}

SwerveModulePosition SwerveModule::getPosition() {
  return {driveMotor.get_position() * driveGearRatio * std::numbers::pi *
              driveWheelDiameter,
          turnMotor.get_position() * turnGearRatio * 1_deg};
}

void SwerveModule::reset() {
  driveMotor.tare_position();
  turnMotor.tare_position();
}

void SwerveModule::setDesiredState(SwerveModuleState &state) {
  auto currentState = getState();
  state.optimize(currentState.angle);
  state.scaleCosine(currentState.angle);

  const auto driveOutput =
      drivePID.calculate(currentState.speed.value(), state.speed.value());

  const auto turnOutput = turnPID.calculate(
      currentState.angle.degrees().value(), state.angle.degrees().value());

  // TODO: check if i need to clamp these properly
  driveMotor.move(driveOutput);
  turnMotor.move(turnOutput);
}

} // namespace libmavnetics

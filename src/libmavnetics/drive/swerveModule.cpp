#include "libmavnetics/drive/swerveModule.hpp"

#include <algorithm>
#include <iostream>
#include <numbers>
#include <ostream>

#include "units/angle.h"
#include "units/length.h"
#include "units/time.h"

namespace libmavnetics {

SwerveModule::SwerveModule(pros::Motor driveMotor, PID drivePIDController,
                           const double driveGearRatio, pros::Motor turnMotor,
                           PID turnPIDController, const double turnGearRatio,
                           const units::meter_t driveWheelDiameter)
    : driveMotor(driveMotor), drivePID(drivePIDController),
      driveGearRatio(driveGearRatio), turnMotor(turnMotor),
      turnPID(turnPIDController), turnGearRatio(turnGearRatio),
      driveWheelDiameter(driveWheelDiameter) {
  // allow the pid to loop between -180 deg and 180 deg
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

units::meters_per_second_t SwerveModule::maxSpeed() {
  switch (driveMotor.get_gearing()) {
  case pros::MotorGearset::red:
    return units::meters_per_second_t{100.0 / 60.0 * driveGearRatio *
                                      driveWheelDiameter.value() *
                                      units::constants::pi};
  case pros::MotorGearset::blue:
    return units::meters_per_second_t{600.0 / 60.0 * driveGearRatio *
                                      driveWheelDiameter.value() *
                                      units::constants::pi};
  case pros::MotorGearset::green:
  case pros::MotorGearset::invalid:
  default:
    return units::meters_per_second_t{200.0 / 60.0 * driveGearRatio *
                                      driveWheelDiameter.value() *
                                      units::constants::pi};
  }
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

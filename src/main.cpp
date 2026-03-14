#include "main.h"

#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <numbers>

#include "globals.hpp"
#include "libmavnetics/utils/rotation2d.hpp"
#include "libmavnetics/utils/util.hpp"
#include "pros/misc.h"
#include "pros/motors.h"
#include "pros/rtos.hpp"
#include "units/angular_velocity.h"
#include "units/math.h"
#include "units/time.h"
#include "units/velocity.h"

// units library: https://github.com/nholthaus/units
// eigen library: https://github.com/LemLib/Eigen
// gcem library: https://github.com/kthohr/gcem

void initialize() {
  tuner.focus();
  tuner.selectPID(1);

  rotateFR.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
  rotateFL.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
  rotateBL.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
  rotateBR.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);

  driveFR.set_brake_mode(pros::E_MOTOR_BRAKE_BRAKE);
  driveFL.set_brake_mode(pros::E_MOTOR_BRAKE_BRAKE);
  driveBL.set_brake_mode(pros::E_MOTOR_BRAKE_BRAKE);
  driveBR.set_brake_mode(pros::E_MOTOR_BRAKE_BRAKE);

  if (imu.reset(true) != 1 && imu.reset(true) != 1 && imu.reset(true) != 1)
    controller.rumble("...");
  else
    controller.rumble("-");
}

void disabled() {}
void competition_initialize() {}
void autonomous() {}

void opcontrol() {
  long lastXPress = -1;

  units::revolutions_per_minute_t moduleWheelSpeed =
      libmavnetics::getRPM(driveCartridge) * driveRatio;

  units::meters_per_second_t maxLinearSpeed =
      moduleWheelSpeed.value() / 60.0_s * driveWheelDiameter * std::numbers::pi;

  units::radians_per_second_t maxRotationalSpeed =
      1_rad * maxLinearSpeed /
      units::math::hypot(track_width / 2.0, wheel_base / 2.0);

  while (true) {
    libmavnetics::Rotation2D heading{-imu.get_heading() * 1_deg + 90_deg};

    units::meters_per_second_t xSpeed =
        static_cast<double>(
            -controller.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y)) *
        maxLinearSpeed / 127.0;

    units::meters_per_second_t ySpeed =
        static_cast<double>(
            controller.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_X)) *
        maxLinearSpeed / 127.0;

    units::radians_per_second_t rotSpeed =
        static_cast<double>(
            controller.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_X)) *
        maxRotationalSpeed / 127.0;

    drive.drive(xSpeed, ySpeed, rotSpeed);

    if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_R1))
      intake.move(127);
    else if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_R2))
      intake.move(-127);
    else
      intake.move(0);

    if (controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_L1))
      ramp.toggle();

    if (lastXPress == -1 &&
        controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_X)) {
      if (intakeBlocker.is_extended()) {
        lastXPress = pros::millis();
      } else {
        intakeBlocker.extend();
      }
    }

    if (lastXPress != -1 && pros::millis() - lastXPress < 500) {
      intake.move(-127);
    } else if (lastXPress != -1) {
      intakeBlocker.retract();
      lastXPress = -1;
    }

    pros::delay(50);
  }
}

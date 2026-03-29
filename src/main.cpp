#include "main.h"

#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <iostream>
#include <ostream>

#include "globals.hpp"
#include "pros/misc.h"
#include "pros/motors.h"
#include "pros/rtos.hpp"
#include "units/angular_velocity.h"
#include "units/velocity.h"

// units library: https://github.com/nholthaus/units
// eigen library: https://github.com/LemLib/Eigen
// gcem library: https://github.com/kthohr/gcem

void initialize() {
  tuner.focus();

  rotateFR.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
  rotateFL.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
  rotateBL.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
  rotateBR.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);

  driveFR.set_brake_mode(pros::E_MOTOR_BRAKE_BRAKE);
  driveFL.set_brake_mode(pros::E_MOTOR_BRAKE_BRAKE);
  driveBL.set_brake_mode(pros::E_MOTOR_BRAKE_BRAKE);
  driveBR.set_brake_mode(pros::E_MOTOR_BRAKE_BRAKE);

  // drivetrain.calibrate();
  imu.reset(true);
  controller.rumble(".");
  // odometry.calibrate(false);

  auto task = pros::Task([=]() {
    while (true) {
      odometry.update();
      auto pose = drivetrain.getPose();
      locator.update(pose);

      std::cout << "heading: " << pose.rotation().degrees().value() << " deg" << std::endl;

      pros::delay(20);
    }
  });

  auto io_task = pros::Task([=]() {
    while (true) {

      double kP, kI, kD;
      std::cin >> kP >> kI >> kD;
      std::cout << kP << ", " << kI << ", " << kD << std::endl;

      FLModule.turnPID.setGains(kP, kI, kD);
      FRModule.turnPID.setGains(kP, kI, kD);
      BLModule.turnPID.setGains(kP, kI, kD);
      BRModule.turnPID.setGains(kP, kI, kD);
    }
  });
}

void disabled() {}
void competition_initialize() {}
void autonomous() {
  logo.focus();
  autonSelector.run_auton();
}

void opcontrol() {
  long lastXPress = -1;

  while (true) {
    units::meters_per_second_t xSpeed =
        static_cast<double>(
            -controller.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y)) *
        maxLinearVelocity / 127.0;

    units::meters_per_second_t ySpeed =
        static_cast<double>(
            controller.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_X)) *
        maxLinearVelocity / 127.0;

    units::radians_per_second_t rotSpeed =
        static_cast<double>(
            controller.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_X)) *
        maxRotationalVelocity / 127.0;

    drivetrain.drive(xSpeed, ySpeed, rotSpeed, true);

    if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_R1)) {
      intakeTop.move(100);
      intakeBottom.move(100);
    } else if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_R2)) {
      intakeTop.move(-100);
      intakeBottom.move(-100);
    } else {
      intakeTop.move(0);
      intakeBottom.move(0);
    }

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

    if (lastXPress != -1 && pros::millis() - lastXPress < 000) {
      intakeBottom.move(-127);
    } else if (lastXPress != -1) {
      intakeBlocker.retract();
      lastXPress = -1;
    }

    pros::delay(20);
  }
}

#include "main.h"

#include "globals.hpp"
#include "pros/llemu.hpp"
#include "pros/misc.h"
#include "pros/motors.h"
#include "pros/rtos.hpp"
#include "robot/utils/rotation2d.hpp"
#include "robot/utils/util.hpp"
#include "units/angular_velocity.h"
#include "units/math.h"
#include "units/time.h"
#include "units/velocity.h"

#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <numbers>

// units library: https://github.com/nholthaus/units

/**
 * Runs initialization code. This occurs as soon as the program is started.
 *
 * All other competition modes are blocked by initialize; it is recommended
 * to keep execution time for this mode under a few seconds.
 */
void initialize() {
  pros::lcd::initialize();

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

/**
 * Runs while the robot is in the disabled state of Field Management System or
 * the VEX Competition Switch, following either autonomous or opcontrol. When
 * the robot is enabled, this task will exit.
 */
void disabled() {}

/**
 * Runs after initialize(), and before autonomous when connected to the Field
 * Management System or the VEX Competition Switch. This is intended for
 * competition-specific initialization routines, such as an autonomous selector
 * on the LCD.
 *
 * This task will exit when the robot is enabled and autonomous or opcontrol
 * starts.
 */
void competition_initialize() {}

/**
 * Runs the user autonomous code. This function will be started in its own task
 * with the default priority and stack size whenever the robot is enabled via
 * the Field Management System or the VEX Competition Switch in the autonomous
 * mode. Alternatively, this function may be called in initialize or opcontrol
 * for non-competition testing purposes.
 *
 * If the robot is disabled or communications is lost, the autonomous task
 * will be stopped. Re-enabling the robot will restart the task, not re-start it
 * from where it left off.
 */
void autonomous() {}

/**
 * Runs the operator control code. This function will be started in its own task
 * with the default priority and stack size whenever the robot is enabled via
 * the Field Management System or the VEX Competition Switch in the operator
 * control mode.
 *
 * If no competition control is connected, this function will run immediately
 * following initialize().
 *
 * If the robot is disabled or communications is lost, the
 * operator control task will be stopped. Re-enabling the robot will restart the
 * task, not resume it from where it left off.
 */
void opcontrol() {
  while (true) {
    libmavnetics::Rotation2D heading{-imu.get_heading() * 1_deg + 90_deg};

    // units::meter_t driveWheelDiameter, track_width, wheel_base
    // std::int32_t controller.get_analog() // returns a number between -127 and
    // 127

    units::revolutions_per_minute_t moduleWheelSpeed =
        libmavnetics::getRPM(driveCartridge) * driveRatio;

    units::meters_per_second_t maxLinearSpeed = moduleWheelSpeed.value() /
                                                60.0_s * driveWheelDiameter *
                                                std::numbers::pi;

    units::radians_per_second_t maxRotationalSpeed =
        1_rad * maxLinearSpeed /
        units::math::hypot(track_width / 2.0, wheel_base / 2.0);

    units::meters_per_second_t xSpeed =
        static_cast<double>(
            -controller.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_X)) *
        maxLinearSpeed / 127.0;

    units::meters_per_second_t ySpeed =
        static_cast<double>(
            controller.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y)) *
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

    if (controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_L2))
      descorer.toggle();

    pros::delay(50);
  }
}

#include "main.h"

#include "globals.hpp"
#include "pros/llemu.hpp"
#include "pros/misc.h"
#include "pros/motors.h"
#include "pros/rtos.hpp"
#include "units/Angle.hpp"
#include "units/units.hpp"
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <iostream>
#include <vector>

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

  std::vector<Number> xVals{};
  std::vector<Angle> yVals{};

  while (true) {
    Angle heading{-imu.get_heading() + 90};

    Number lx = -controller.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_X);
    Number ly = controller.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y);
    Number rx = controller.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_X);

    xVals.push_back(rx);
    yVals.push_back(heading);

    drive.driverControl(heading, lx, ly, rx, false);

    if (controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_B)) {
      break;
    }

    pros::delay(50);
  }

  std::cout << "xVals = [";
  for (Number x : xVals) {
    std::cout << x.convert(1) << ", "; 
  }
  std::cout << "]\n";

  std::cout << "yVals = [";
  for (Angle y : yVals) {
    std::cout << y.convert(1_stRad) << ", "; 
  }
  std::cout << "]\n";
}
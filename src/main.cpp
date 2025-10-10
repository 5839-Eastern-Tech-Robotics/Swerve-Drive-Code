#include "main.h"

#include "globals.hpp"
#include "pros/abstract_motor.hpp"
#include "pros/llemu.hpp"
#include "pros/misc.h"
#include "pros/motors.h"
#include "pros/rtos.hpp"
#include "units/Angle.hpp"
#include "units/units.hpp"
#include <algorithm>
#include <cmath>
#include <cstdio>
#include <cstdlib>

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

  while (true) {
    Number lx = -controller.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_X);
    Number ly = controller.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y);
    Number rx = controller.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_X);

    drive.holonomic(lx, ly, rx);

    pros::delay(50);
  }

  return;



  // get max theoretical rotation speed
  float driveRPM =
      driveCartridge == pros::MotorGears::rpm_100 ? 100
          : (driveCartridge == pros::MotorGears::rpm_200 ? 200
                 : (driveCartridge == pros::MotorGears::rpm_600 ? 600 : 0));
  float theoreticalMaxRotVel = 3 * driveRPM * driveRatio * driveWheelDiameter / driveRadius;

  int prevtime = pros::millis() - 50;
  float prevRotation = imu.get_heading();
  while (true) {
    // get deltatime and log current rotation speed
    float deltaTime = (pros::millis() - prevtime) / 1000.0;
    float curRotVel = (imu.get_heading() - prevRotation) / deltaTime;
    printf("Max = %f deg/sec, current = %f deg/sec, dt = %f, heading = %f deg\n", theoreticalMaxRotVel, curRotVel, deltaTime, imu.get_heading());
    prevtime = pros::millis();
    prevRotation = imu.get_heading();

    // get controller input
    float fwd = controller.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y)  /  127.0;
    float str = controller.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_X)  / -127.0;
    float rot = controller.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_X) / -127.0;

    // reset heading
    if (controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_X))
      imu.set_heading(0);

    // convert input from local frame of ref to global frame of ref
    float theta = imu.get_heading() * M_PI / 180;
    float tmp = fwd * std::cos(theta) - str * std::sin(theta);
    str = fwd * std::sin(theta) + str * std::cos(theta);
    fwd = tmp;

    // drive.holonomic(fwd, str, rot);

    if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_R1)) {
      intake.move(127);
      liftBelt.move(127);
    } else if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_R2)) {
      intake.move(-127);
      liftBelt.move(-127);
    } else {
      intake.move(0);
      liftBelt.move(0);
    }

    // pros::delay(50);
    // continue;

    // save repeated calculations
    float A = str - rot * driveLength / driveRadius;
    float B = str + rot * driveLength / driveRadius;
    float C = fwd + rot * driveWidth / driveRadius;
    float D = fwd - rot * driveWidth / driveRadius;

    // calculate magnitude of drive vector
    float FRSpeed = std::sqrt(B * B + C * C);
    float FLSpeed = std::sqrt(B * B + D * D);
    float BLSpeed = std::sqrt(A * A + D * D);
    float BRSpeed = std::sqrt(A * A + C * C);

    // scale drive vectors to 0-1 range
    float maxSpeed = std::max({FRSpeed, FLSpeed, BLSpeed, BRSpeed});
    if (maxSpeed > 1) {
      FRSpeed /= maxSpeed;
      FLSpeed /= maxSpeed;
      BLSpeed /= maxSpeed;
      BRSpeed /= maxSpeed;
    }

    // calculate heading for drive vector
    float FRAngle = std::atan2(B, C) * 180 / M_PI / rotateRatio;
    float FLAngle = std::atan2(B, D) * 180 / M_PI / rotateRatio;
    float BLAngle = std::atan2(A, D) * 180 / M_PI / rotateRatio;
    float BRAngle = std::atan2(A, C) * 180 / M_PI / rotateRatio;

    // calculate error to calculate minimum rotation later
    float FRError = FRAngle - std::remainder(rotateFR.get_position(), (360 / rotateRatio));
    float FLError = FLAngle - std::remainder(rotateFL.get_position(), (360 / rotateRatio));
    float BLError = BLAngle - std::remainder(rotateBL.get_position(), (360 / rotateRatio));
    float BRError = BRAngle - std::remainder(rotateBR.get_position(), (360 / rotateRatio));

    // Get smallest rotation algo (kinda skuffed lol)
    FRError = std::remainder(FRError, (360 / rotateRatio));
    FLError = std::remainder(FLError, (360 / rotateRatio));
    BLError = std::remainder(BLError, (360 / rotateRatio));
    BRError = std::remainder(BRError, (360 / rotateRatio));

    // Flip rotation and velocity direction
    if (FRError > 90 / rotateRatio) {
      FRError -= 180 / rotateRatio;
      FRSpeed *= -1;
    }
    if (FRError < -90 / rotateRatio) {
      FRError += 180 / rotateRatio;
      FRSpeed *= -1;
    }

    if (FLError > 90 / rotateRatio) {
      FLError -= 180 / rotateRatio;
      FLSpeed *= -1;
    }
    if (FLError < -90 / rotateRatio) {
      FLError += 180 / rotateRatio;
      FLSpeed *= -1;
    }

    if (BLError > 90 / rotateRatio) {
      BLError -= 180 / rotateRatio;
      BLSpeed *= -1;
    }
    if (BLError < -90 / rotateRatio) {
      BLError += 180 / rotateRatio;
      BLSpeed *= -1;
    }

    if (BRError > 90 / rotateRatio) {
      BRError -= 180 / rotateRatio;
      BRSpeed *= -1;
    }
    if (BRError < -90 / rotateRatio) {
      BRError += 180 / rotateRatio;
      BRSpeed *= -1;
    }

    // actually move to the angle using PID, TODO: retune PID
    rotateFR.move(FRPID.update(FRError));
    rotateFL.move(FLPID.update(FLError));
    rotateBL.move(BLPID.update(BLError));
    rotateBR.move(BRPID.update(BRError));

    // drive the wheels at the right velocity
    driveFR.move(FRSpeed * 127);
    driveFL.move(FLSpeed * 127);
    driveBL.move(BLSpeed * 127);
    driveBR.move(BRSpeed * 127);
    
    pros::delay(50);
  }
}

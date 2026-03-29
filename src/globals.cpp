#include "globals.hpp"

#include "autons.hpp"
#include "gcem.hpp"
#include "gcem_incl/sin.hpp"
#include "libmavnetics/drive/odometry.hpp"
#include "libmavnetics/drive/swerveDrive.hpp"
#include "libmavnetics/drive/swerveModule.hpp"
#include "libmavnetics/ui/views/autonSelector.hpp"
#include "libmavnetics/ui/views/debug.hpp"
#include "libmavnetics/ui/views/image.hpp"
#include "libmavnetics/ui/views/locator.hpp"
#include "libmavnetics/ui/views/pidTuner.hpp"
#include "libmavnetics/ui/views/swerve.hpp"
#include "libmavnetics/utils/pid.hpp"
#include "pros/abstract_motor.hpp"
#include "pros/adi.hpp"
#include "pros/imu.hpp"
#include "pros/misc.h"
#include "pros/misc.hpp"
#include "pros/motor_group.hpp"
#include "pros/motors.hpp"
#include "pros/rotation.hpp"
#include "units/length.h"

pros::Motor intakeBottom{-6};
pros::Motor intakeTop{-7};
pros::adi::Pneumatics intakeBlocker{'B', true, true};
pros::adi::Pneumatics ramp{'C', false};

const pros::MotorCartridge driveCartridge = pros::MotorCartridge::blue;
const double driveRatio = 1.0 / 2.0;
const units::meter_t driveWheelDiameter = 2_in;
libmavnetics::PID driveMotorPID{250, 0, 0};

const pros::MotorCartridge rotateCartridge = pros::MotorCartridge::green;
const double rotateRatio = 12.0 / 66.0;
// libmavnetics::PID rotateMotorPID{15, 0, 0.5};
libmavnetics::PID rotateMotorPID{5, 0, 0};

const units::meter_t track_width = 13.25_in;
const units::meter_t wheel_base = 11.25_in;
const units::revolutions_per_minute_t moduleWheelSpeed =
    libmavnetics::getRPM(driveCartridge) * driveRatio;
const units::meters_per_second_t maxLinearVelocity =
    moduleWheelSpeed.value() / 60.0_s * driveWheelDiameter * std::numbers::pi;
const units::radians_per_second_t maxRotationalVelocity =
    1_rad * maxLinearVelocity /
    units::math::hypot(track_width / 2.0, wheel_base / 2.0);

pros::Motor driveBL(-5, driveCartridge, pros::MotorEncoderUnits::degrees);
pros::Motor rotateBL{18, rotateCartridge, pros::MotorEncoderUnits::degrees};
libmavnetics::SwerveModule BLModule{
    driveBL,          {driveMotorPID}, driveRatio,        rotateBL,
    {rotateMotorPID}, rotateRatio,     driveWheelDiameter};

pros::Motor driveBR{-4, driveCartridge, pros::MotorEncoderUnits::degrees};
pros::Motor rotateBR{2, rotateCartridge, pros::MotorEncoderUnits::degrees};
libmavnetics::SwerveModule BRModule{
    driveBR,          {driveMotorPID}, driveRatio,        rotateBR,
    {rotateMotorPID}, rotateRatio,     driveWheelDiameter};

pros::Motor driveFL{-19, driveCartridge, pros::MotorEncoderUnits::degrees};
pros::Motor rotateFL{17, rotateCartridge, pros::MotorEncoderUnits::degrees};
libmavnetics::SwerveModule FLModule{
    driveFL,          {driveMotorPID}, driveRatio,        rotateFL,
    {rotateMotorPID}, rotateRatio,     driveWheelDiameter};

pros::Motor driveFR{-12, driveCartridge, pros::MotorEncoderUnits::degrees};
pros::Motor rotateFR(11, rotateCartridge, pros::MotorEncoderUnits::degrees);
libmavnetics::SwerveModule FRModule{
    driveFR,          {driveMotorPID}, driveRatio,        rotateFR,
    {rotateMotorPID}, rotateRatio,     driveWheelDiameter};

pros::IMU imu{13};
pros::Rotation verticalSensor{15};
pros::Rotation horizontalSensor{14};
libmavnetics::OdometryModule verticalModule{&verticalSensor, 2_in, 6_in};
libmavnetics::OdometryModule horizontalModule{&horizontalSensor, 2_in, 6_in};
libmavnetics::Odometry odometry{&verticalModule, &horizontalModule, &imu};

libmavnetics::PID linearXPID{1, 0, 0};
libmavnetics::PID linearYPID{1, 0, 0};
libmavnetics::PID rotationalPID{1, 0, 0};
libmavnetics::PID straighteningPID{1, 0, 0};

libmavnetics::SwerveDrive drivetrain{{&FLModule, &FRModule, &BLModule, &BRModule},
                                     track_width,
                                     wheel_base,
                                     &odometry,
                                     &linearXPID,
                                     &linearYPID,
                                     &rotationalPID,
                                     &straighteningPID};

pros::Controller controller{pros::E_CONTROLLER_MASTER};

void update_angle_pid() {
  // auto current_angle = rotateFL.get_position() * rotateRatio * 1_deg;
  // const auto turn_output = driveMotorPID.calculate(current_angle.value(), 90);
  // rotateFL.move(turn_output);
}

libmavnetics::gui::AutonSelector autonSelector({
    {"Red Left Side", red_left_side, "", 0},
    {"Red Right Side", red_right_side, "", 0},
    {"Blue Left Side", blue_left_side, "", 240},
    {"Blue Right Side", blue_right_side, "", 240},
    {"Skills", skills, "", 120},
});
libmavnetics::gui::DebugConsole console{};
libmavnetics::gui::OdomLocator locator{};
libmavnetics::gui::ImageView logo{"Logo", &eths_logo};
libmavnetics::gui::PIDTuner tuner({
    {"Module Angle", &FLModule.turnPID, update_angle_pid},
});

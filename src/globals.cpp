#include "globals.hpp"

#include "libmavnetics/drive/odometry.hpp"
#include "libmavnetics/drive/swerveDrive.hpp"
#include "libmavnetics/drive/swerveModule.hpp"
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

pros::IMU imu{14};
pros::MotorGroup intake{6, 7};
pros::adi::Pneumatics descorer{'B', false};
pros::adi::Pneumatics ramp{'C', false};

const pros::MotorCartridge driveCartridge = pros::MotorCartridge::blue;
const double driveRatio = 1.0 / 2.0;
const units::meter_t driveWheelDiameter = 2_in;
libmavnetics::PID driveMotorPID{1, 0, 0};

const pros::MotorCartridge rotateCartridge = pros::MotorCartridge::green;
const double rotateRatio = 12.0 / 66.0;
libmavnetics::PID rotateMotorPID{1, 0, 0};

const units::meter_t track_width = 13.25_in;
const units::meter_t wheel_base = 11.25_in;

pros::Motor driveBL(10, driveCartridge, pros::MotorEncoderUnits::degrees);
pros::Motor rotateBL{8, rotateCartridge, pros::MotorEncoderUnits::degrees};
libmavnetics::SwerveModule BLModule{
    driveBL,          {driveMotorPID}, driveRatio,        rotateBL,
    {rotateMotorPID}, rotateRatio,     driveWheelDiameter};

pros::Motor driveBR{4, driveCartridge, pros::MotorEncoderUnits::degrees};
pros::Motor rotateBR{2, rotateCartridge, pros::MotorEncoderUnits::degrees};
libmavnetics::SwerveModule BRModule{
    driveBR,          {driveMotorPID}, driveRatio,        rotateBR,
    {rotateMotorPID}, rotateRatio,     driveWheelDiameter};

pros::Motor driveFL{18, driveCartridge, pros::MotorEncoderUnits::degrees};
pros::Motor rotateFL{17, rotateCartridge, pros::MotorEncoderUnits::degrees};
libmavnetics::SwerveModule FLModule{
    driveFL,          {driveMotorPID}, driveRatio,        rotateFL,
    {rotateMotorPID}, rotateRatio,     driveWheelDiameter};

pros::Motor driveFR{12, driveCartridge, pros::MotorEncoderUnits::degrees};
pros::Motor rotateFR(11, rotateCartridge, pros::MotorEncoderUnits::degrees);
libmavnetics::SwerveModule FRModule{
    driveFR,          {driveMotorPID}, driveRatio,        rotateFR,
    {rotateMotorPID}, rotateRatio,     driveWheelDiameter};

pros::Rotation verticalSensor{13};
libmavnetics::OdometryModule verticalModule{&verticalSensor, 2_in, 6_in};

pros::Rotation horizontalSensor{14};
libmavnetics::OdometryModule horizontalModule{&horizontalSensor, 2_in, 6_in};

libmavnetics::Odometry odometry{&verticalModule, &horizontalModule, &imu};
libmavnetics::SwerveDrive drive{{FLModule, FRModule, BLModule, BRModule},
                                track_width,
                                wheel_base,
                                &odometry};

pros::Controller controller{pros::E_CONTROLLER_MASTER};

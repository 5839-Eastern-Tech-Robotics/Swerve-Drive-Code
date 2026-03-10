#pragma once

#include "pros/abstract_motor.hpp"
#include "pros/adi.hpp"
#include "pros/imu.hpp"
#include "pros/misc.hpp"
#include "pros/motor_group.hpp"
#include "pros/motors.hpp"
#include "robot/utils/pid.hpp"
#include "units/length.h"

extern pros::IMU imu;
extern pros::MotorGroup intake;
extern pros::adi::Pneumatics descorer;
extern pros::adi::Pneumatics ramp;

extern pros::MotorCartridge driveCartridge;
extern double driveRatio;
extern units::meter_t driveWheelDiameter;

extern pros::MotorCartridge rotateCartridge;
extern double rotateRatio;
extern libmavnetics::PID rotateMotorPID;

extern pros::Motor driveBL;
extern pros::Motor rotateBL;
extern libmavnetics::PID BLPID;
// extern libmavnetics::SwerveModule BLModule;

extern pros::Motor driveBR;
extern pros::Motor rotateBR;
extern libmavnetics::PID BRPID;
// extern libmavnetics::SwerveModule BRModule;

extern pros::Motor driveFL;
extern pros::Motor rotateFL;
extern libmavnetics::PID FLPID;
// extern libmavnetics::SwerveModule FLModule;

extern pros::Motor driveFR;
extern pros::Motor rotateFR;
extern libmavnetics::PID FRPID;
// extern libmavnetics::SwerveModule FRModule;

extern libmavnetics::PID stabilityPID;
extern libmavnetics::PID drivePID;
extern libmavnetics::PID strafePID;
extern libmavnetics::PID rotatePID;

extern pros::Controller controller;

// extern libmavnetics::SwerveDrive drive;

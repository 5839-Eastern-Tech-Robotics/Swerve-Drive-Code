#pragma once

#include "pros/abstract_motor.hpp"
#include "pros/imu.hpp"
#include "pros/misc.hpp"
#include "pros/motors.hpp"
#include "robot/drive/swerveDrive.hpp"
#include "robot/utils/pid.hpp"
#include "units/units.hpp"

extern pros::IMU imu;
extern libmavnetics::PID rotateMotorPID;

extern pros::Motor driveBL;
extern pros::Motor rotateBL;
extern libmavnetics::PID BLPID;

extern pros::Motor driveBR;
extern pros::Motor rotateBR;
extern libmavnetics::PID BRPID;

extern pros::Motor driveFL;
extern pros::Motor rotateFL;
extern libmavnetics::PID FLPID;

extern pros::Motor driveFR;
extern pros::Motor rotateFR;
extern libmavnetics::PID FRPID;

extern pros::MotorCartridge driveCartridge;
extern Number driveRatio;
extern Length driveWheelDiameter;

extern pros::MotorCartridge rotateCartridge;
extern Number rotateRatio;

extern Length driveLength;
extern Length driveWidth;
extern Length driveRadius;

extern pros::Controller controller;

extern libmavnetics::SwerveModule FRModule;
extern libmavnetics::SwerveModule FLModule;
extern libmavnetics::SwerveModule BLModule;
extern libmavnetics::SwerveModule BRModule;
extern libmavnetics::SwerveDrive drive;
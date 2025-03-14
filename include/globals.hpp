#pragma once

#include "pros/imu.hpp"
#include "pros/misc.hpp"
#include "pros/motors.hpp"
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

extern Length driveLength;
extern Length driveWidth;
extern Length driveRadius;
extern Number rotateRatio;

extern pros::Controller controller;
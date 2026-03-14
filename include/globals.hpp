#pragma once

#include "libmavnetics/drive/swerveDrive.hpp"
#include "libmavnetics/drive/swerveModule.hpp"
#include "libmavnetics/drive/swerveModule.hpp"
#include "libmavnetics/ui/views/pidTuner.hpp"
#include "libmavnetics/utils/pid.hpp"
#include "pros/abstract_motor.hpp"
#include "pros/adi.hpp"
#include "pros/imu.hpp"
#include "pros/misc.hpp"
#include "pros/motor_group.hpp"
#include "pros/motors.hpp"
#include "units/length.h"

extern pros::IMU imu;
extern pros::MotorGroup intake;
extern pros::adi::Pneumatics intakeBlocker;
extern pros::adi::Pneumatics ramp;

extern const pros::MotorCartridge driveCartridge;
extern const double driveRatio;
extern const units::meter_t driveWheelDiameter;
extern libmavnetics::PID driveMotorPID;

extern const pros::MotorCartridge rotateCartridge;
extern const double rotateRatio;
extern libmavnetics::PID rotateMotorPID;

extern const units::meter_t track_width;
extern const units::meter_t wheel_base;

extern pros::Motor driveBL;
extern pros::Motor rotateBL;
extern libmavnetics::SwerveModule BLModule;

extern pros::Motor driveBR;
extern pros::Motor rotateBR;
extern libmavnetics::SwerveModule BRModule;

extern pros::Motor driveFL;
extern pros::Motor rotateFL;
extern libmavnetics::SwerveModule FLModule;

extern pros::Motor driveFR;
extern pros::Motor rotateFR;
extern libmavnetics::SwerveModule FRModule;

extern libmavnetics::SwerveDrive drive;

extern pros::Controller controller;

extern libmavnetics::gui::PIDTuner tuner;

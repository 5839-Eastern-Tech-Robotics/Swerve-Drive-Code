#pragma once

#include "liblvgl/widgets/image/lv_image.h"
#include "libmavnetics/drive/swerveDrive.hpp"
#include "libmavnetics/drive/swerveModule.hpp"
#include "libmavnetics/ui/views/debug.hpp"
#include "libmavnetics/ui/views/image.hpp"
#include "libmavnetics/ui/views/locator.hpp"
#include "libmavnetics/ui/views/pidTuner.hpp"
#include "libmavnetics/ui/views/swerve.hpp"
#include "libmavnetics/utils/pid.hpp"
#include "pros/abstract_motor.hpp"
#include "pros/adi.hpp"
#include "pros/imu.hpp"
#include "pros/misc.hpp"
#include "pros/motor_group.hpp"
#include "pros/motors.hpp"
#include "units/angular_velocity.h"
#include "units/length.h"
#include "units/velocity.h"

extern pros::Motor intakeBottom;
extern pros::Motor intakeTop;
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
extern const units::meters_per_second_t maxLinearVelocity;
extern const units::radians_per_second_t maxRotationalVelocity;

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

extern pros::IMU imu;
extern pros::Rotation verticalSensor;
extern pros::Rotation horizontalSensor;
extern libmavnetics::OdometryModule verticalModule;
extern libmavnetics::OdometryModule horizontalModule;
extern libmavnetics::Odometry odometry;

extern libmavnetics::PID linearXPID;
extern libmavnetics::PID linearYPID;
extern libmavnetics::PID rotationalPID;
extern libmavnetics::PID straighteningPID;

extern libmavnetics::SwerveDrive drivetrain;

extern pros::Controller controller;

extern libmavnetics::gui::AutonSelector autonSelector;
extern libmavnetics::gui::DebugConsole console;
extern libmavnetics::gui::OdomLocator locator;
extern libmavnetics::gui::PIDTuner tuner;

extern const lv_image_dsc_t eths_logo;
extern libmavnetics::gui::ImageView logo;

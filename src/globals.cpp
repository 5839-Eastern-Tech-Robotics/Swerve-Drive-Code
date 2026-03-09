#include "globals.hpp"

#include "pros/abstract_motor.hpp"
#include "pros/adi.hpp"
#include "pros/imu.hpp"
#include "pros/misc.h"
#include "pros/misc.hpp"
#include "pros/motor_group.hpp"
#include "pros/motors.hpp"
#include "robot/drive/swerveDrive.hpp"
#include "robot/utils/pid.hpp"
#include "units/units.hpp"

pros::IMU imu{14};
pros::MotorGroup intake{6, 7};
pros::adi::Pneumatics descorer{'B', false};
pros::adi::Pneumatics ramp{'C', false};

pros::MotorCartridge driveCartridge = pros::MotorCartridge::blue;
Number driveRatio = 1.0 / 2.0;
Length driveWheelDiameter = 2_in;

pros::MotorCartridge rotateCartridge = pros::MotorCartridge::green;
Number rotateRatio =  12.0 / 66.0;
libmavnetics::PID rotateMotorPID{
	1,
	0, 
	0
};


pros::Motor driveBL(10, driveCartridge, pros::MotorEncoderUnits::degrees);
pros::Motor rotateBL{8, rotateCartridge, pros::MotorEncoderUnits::degrees};
libmavnetics::PID BLPID{rotateMotorPID};
libmavnetics::SwerveModule BLModule{driveBL, rotateBL, BLPID, {-5.25_in, -6.75_in}, driveWheelDiameter, driveRatio, rotateRatio};

pros::Motor driveBR{4, driveCartridge, pros::MotorEncoderUnits::degrees};
pros::Motor rotateBR{2, rotateCartridge, pros::MotorEncoderUnits::degrees};
libmavnetics::PID BRPID{rotateMotorPID};
libmavnetics::SwerveModule BRModule{driveBR, rotateBR, BRPID, {5.25_in, -6.75_in}, driveWheelDiameter, driveRatio, rotateRatio};

pros::Motor driveFL{18, driveCartridge, pros::MotorEncoderUnits::degrees};
pros::Motor rotateFL{17, rotateCartridge, pros::MotorEncoderUnits::degrees};
libmavnetics::PID FLPID{rotateMotorPID};
libmavnetics::SwerveModule FLModule{driveFL, rotateFL, FLPID, {-5.25_in, 6.75_in}, driveWheelDiameter, driveRatio, rotateRatio};

pros::Motor driveFR{12, driveCartridge, pros::MotorEncoderUnits::degrees};
pros::Motor rotateFR(11, rotateCartridge, pros::MotorEncoderUnits::degrees);
libmavnetics::PID FRPID{rotateMotorPID};
libmavnetics::SwerveModule FRModule{driveFR, rotateFR, FRPID, {5.25_in, 6.75_in}, driveWheelDiameter, driveRatio, rotateRatio};

libmavnetics::SwerveDrive drive{{BLModule, BRModule, FLModule, FRModule}, {0, 0, 0}};

pros::Controller controller{pros::E_CONTROLLER_MASTER};

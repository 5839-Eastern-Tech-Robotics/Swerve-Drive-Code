#include "globals.hpp"

#include "pros/abstract_motor.hpp"
#include "pros/imu.hpp"
#include "pros/misc.h"
#include "pros/misc.hpp"
#include "pros/motor_group.hpp"
#include "pros/motors.hpp"
#include "robot/drive/swerveDrive.hpp"
#include "robot/utils/pid.hpp"
#include "units/units.hpp"

pros::IMU imu{18};

libmavnetics::PID rotateMotorPID{
    1,
    0, 
    0
};

pros::MotorGroup intake{5, -4};
pros::Motor liftBelt{6};

pros::Motor driveBL(10, driveCartridge, pros::MotorEncoderUnits::degrees);
pros::Motor rotateBL{9, rotateCartridge, pros::MotorEncoderUnits::degrees};
libmavnetics::PID BLPID{rotateMotorPID};
libmavnetics::SwerveModule BLModule{driveBL, rotateBL, BLPID, {-5.25_in, -6.75_in}, 2_in, 2.0/3.0, 2.0/3.0};

pros::Motor driveBR{1, driveCartridge, pros::MotorEncoderUnits::degrees};
pros::Motor rotateBR{2, pros::MotorGears::green, pros::MotorEncoderUnits::degrees};
libmavnetics::PID BRPID{rotateMotorPID};
libmavnetics::SwerveModule BRModule{driveBR, rotateBR, BRPID, {5.25_in, -6.75_in}, 2_in, 2.0/3.0, 2.0/3.0};

pros::Motor driveFL{17, driveCartridge, pros::MotorEncoderUnits::degrees};
pros::Motor rotateFL{19, pros::MotorGears::green, pros::MotorEncoderUnits::degrees};
libmavnetics::PID FLPID{rotateMotorPID};
libmavnetics::SwerveModule FLModule{driveFL, rotateFL, FLPID, {-5.25_in, 6.75_in}, 2_in, 2.0/3.0, 2.0/3.0};

pros::Motor driveFR{11, driveCartridge, pros::MotorEncoderUnits::degrees};
pros::Motor rotateFR(12, pros::MotorGears::green, pros::MotorEncoderUnits::degrees);
libmavnetics::PID FRPID{rotateMotorPID};
libmavnetics::SwerveModule FRModule{driveFR, rotateFR, FRPID, {5.25_in, 6.75_in}, 2_in, 2.0/3.0, 2.0/3.0};

pros::MotorCartridge driveCartridge = pros::MotorCartridge::green;
Number driveRatio = 2.0 / 3.0;
Length driveWheelDiameter = 2_in;

pros::MotorCartridge rotateCartridge = pros::MotorCartridge::green;
Number rotateRatio =  2.0 / 3.0;

Length driveWidth = 10.5_in;
Length driveLength = 13.5_in;
Length driveRadius = units::sqrt(driveLength * driveLength / 4.0 + driveWidth * driveWidth / 4.0);

libmavnetics::SwerveDrive drive{{BLModule, BRModule, FLModule, FRModule}, {0, 0, 0}};

pros::Controller controller{pros::E_CONTROLLER_MASTER};

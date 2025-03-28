#include "globals.hpp"

#include "pros/abstract_motor.hpp"
#include "pros/imu.hpp"
#include "pros/misc.h"
#include "pros/misc.hpp"
#include "pros/motors.hpp"
#include "robot/utils/pid.hpp"
#include "units/units.hpp"

pros::IMU imu{18};

libmavnetics::PID rotateMotorPID{
    1,
    0, 
    0
};

pros::Motor driveBL(10, driveCartridge, pros::MotorEncoderUnits::degrees);
pros::Motor rotateBL{9, rotateCartridge, pros::MotorEncoderUnits::degrees};
libmavnetics::PID BLPID{rotateMotorPID};

pros::Motor driveBR{1, driveCartridge, pros::MotorEncoderUnits::degrees};
pros::Motor rotateBR{2, pros::MotorGears::green, pros::MotorEncoderUnits::degrees};
libmavnetics::PID BRPID{rotateMotorPID};

pros::Motor driveFL{20, driveCartridge, pros::MotorEncoderUnits::degrees};
pros::Motor rotateFL{19, pros::MotorGears::green, pros::MotorEncoderUnits::degrees};
libmavnetics::PID FLPID{rotateMotorPID};

pros::Motor driveFR{11, driveCartridge, pros::MotorEncoderUnits::degrees};
pros::Motor rotateFR(12, pros::MotorGears::green, pros::MotorEncoderUnits::degrees);
libmavnetics::PID FRPID{rotateMotorPID};

pros::MotorCartridge driveCartridge = pros::MotorCartridge::green;
Number driveRatio = 2.0 / 3.0;
Length driveWheelDiameter = 2_in;

pros::MotorCartridge rotateCartridge = pros::MotorCartridge::green;
Number rotateRatio =  2.0 / 3.0;

Length driveWidth = 10.5_in;
Length driveLength = 13.5_in;
Length driveRadius = units::sqrt(driveLength * driveLength / 4.0 + driveWidth * driveWidth / 4.0);

pros::Controller controller{pros::E_CONTROLLER_MASTER};
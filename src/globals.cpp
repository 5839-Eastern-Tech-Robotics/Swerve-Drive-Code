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

pros::Motor driveBL(10, pros::MotorGears::blue, pros::MotorEncoderUnits::degrees);
pros::Motor rotateBL{9, pros::MotorGears::green, pros::MotorEncoderUnits::degrees};
libmavnetics::PID BLPID{rotateMotorPID};

pros::Motor driveBR{1, pros::MotorGears::blue, pros::MotorEncoderUnits::degrees};
pros::Motor rotateBR{2, pros::MotorGears::green, pros::MotorEncoderUnits::degrees};
libmavnetics::PID BRPID{rotateMotorPID};

pros::Motor driveFL{20, pros::MotorGears::blue, pros::MotorEncoderUnits::degrees};
pros::Motor rotateFL{19, pros::MotorGears::green, pros::MotorEncoderUnits::degrees};
libmavnetics::PID FLPID{rotateMotorPID};

pros::Motor driveFR{11, pros::MotorGears::blue, pros::MotorEncoderUnits::degrees};
pros::Motor rotateFR(12, pros::MotorGears::green, pros::MotorEncoderUnits::degrees);
libmavnetics::PID FRPID{rotateMotorPID};

Length driveWidth = 10.5_in;
Length driveLength = 13.5_in;
Length driveRadius = units::sqrt(driveLength * driveLength + driveWidth * driveWidth);
Number rotateRatio = (float) 2 / 3;

pros::Controller controller{pros::E_CONTROLLER_MASTER};
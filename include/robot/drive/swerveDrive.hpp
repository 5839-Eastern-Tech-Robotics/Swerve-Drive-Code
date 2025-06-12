#pragma once

#include "pros/motors.hpp"
#include "robot/utils/pid.hpp"
#include "units/Angle.hpp"
#include "units/Vector2D.hpp"
#include "units/units.hpp"
#include <cstdint>
#include <vector>

namespace libmavnetics {

struct SwerveModule {
    pros::Motor driveMotor;
    pros::Motor rotateMotor;
    libmavnetics::PID rotatePID;
    units::Vector2D<Length> locator;
    Length wheelDiameter = 2.75_in;
    Number driveRatio = 1;
    Number rotateRatio = 1;

    /**
     * @brief Get the current angle the module is facing
     * 
     * @return Angle the angle between 0 and 360 degrees
     */
    Angle getModuleAngle();

    /**
     * @brief rotates the pod to the specified angle constrained to 360
     * takes the shortest path possible
     * 
     * @param angle 
     * @return -1 if motor velocity needs to be flipped else 1
     */
    int rotateTo(Angle angle);

    /**
     * @brief spin the motors at the desired velocity
     * 
     * @param vel 0-127
     */
    void move(int8_t vel);
};

class SwerveDrive {
public:
    SwerveDrive(std::vector<SwerveModule> modules);

    void holonomic(Number fwdVel, Number strVel, Number trnVel);
    void globalHolonomic(Angle heading, Number fwdVel, Number strVel, Number driveLength);

private:
    std::vector<SwerveModule> modules;
};
} // namespace libmavnetics

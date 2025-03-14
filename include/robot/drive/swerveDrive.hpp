#pragma once

#include "pros/motors.hpp"
#include "units/Angle.hpp"
#include "units/Vector2D.hpp"
#include "units/units.hpp"
#include <cstdint>
#include <vector>

namespace libmavnetics {

struct SwerveModule {
    pros::Motor driveMotor;
    pros::Motor rotateMotor;
    units::Vector2D<Length> locator;
    Length wheelDiameter = 2.75_in;
    Number driveRatio = 1;
    Number rotateRatio = 1;

    void rotateTo(Angle angle);
    void move(int8_t vel);
    void moveVector(units::Vector2D<Number> vec);
};

class SwerveDrive {
public:
    SwerveDrive(std::vector<SwerveModule> modules, Length driveWidth, Length driveLength);

    void holonomic(Number fwdVel, Number strVel, Number trnVel);

private:
    std::vector<SwerveModule> modules;
    Length driveLength;
    Length driveWidth;
};
} // namespace libmavnetics
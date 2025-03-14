#include "robot/drive/swerveDrive.hpp"
#include "globals.hpp"
#include "units/Angle.hpp"
#include "units/Vector2D.hpp"
#include "units/units.hpp"
#include <cstdint>

namespace libmavnetics {
void SwerveModule::rotateTo(Angle angle) {}

void SwerveModule::move(int8_t vel) { driveMotor.move(vel); }

void SwerveModule::moveVector(units::Vector2D<Number> vec) {
  rotateTo(vec.angleTo({0, 0}));
  move(vec.magnitude());
}

SwerveDrive::SwerveDrive(std::vector<SwerveModule> modules, Length driveWidth,
                         Length driveLength)
    : modules(modules), driveWidth(driveWidth), driveLength(driveLength) {}

void SwerveDrive::holonomic(Number fwdVel, Number strVel, Number trnVel) {}
} // namespace libmavnetics
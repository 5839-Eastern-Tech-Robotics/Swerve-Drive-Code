#include "robot/drive/swerveDrive.hpp"
#include "units/Angle.hpp"
#include "units/Vector2D.hpp"
#include "units/units.hpp"
#include <cstdint>

namespace libmavnetics {

Angle SwerveModule::getModuleAngle() {
  return units::constrainAngle360(rotateMotor.get_position() * 1_stDeg * rotateRatio);
}

int SwerveModule::rotateTo(Angle angle) {
  int out = 1;
  Angle error = angle - getModuleAngle();
  error = units::constrainAngle180(error);

  if (error > 90_stDeg) {
    error -= 180_stDeg;
    out *= -1;
  }

  if (error < -90_stDeg) {
    error += 180_stDeg;
    out *= -1;
  }

  rotateMotor.move(rotatePID.update(error.convert(1_stDeg)));
  return out;
}

void SwerveModule::move(int8_t vel) {
  driveMotor.move(vel);
}

SwerveDrive::SwerveDrive(std::vector<SwerveModule> modules)
    : modules(modules) {}

void SwerveDrive::holonomic(Number fwdVel, Number strVel, Number trnVel) {

    Number maxSpeed = 0;
    std::vector<units::Vector2D<Number>> wheelVectors;

    for (const libmavnetics::SwerveModule& module : modules) {
        units::Vector2D<Number> rotationVec = {
             module.locator.y.convert(1_in) * trnVel,
             module.locator.x.convert(1_in) * trnVel
        };

        units::Vector2D<Number> wheelVec = {
            strVel + rotationVec.x,
            fwdVel + rotationVec.y
        };

        wheelVectors.push_back(wheelVec);

        maxSpeed = std::max(maxSpeed, wheelVec.magnitude());
    }

    Number scale = maxSpeed > 127 ? (127 / maxSpeed) : 1;
    
    for (size_t i = 0; i < modules.size(); ++i) {
        libmavnetics::SwerveModule& module = modules[i];
        units::Vector2D<Number> wheelVec = wheelVectors[i];

        std::cout << "(" << module.locator.x << ", " << module.locator.y << "): ";
        std::cout << "(" << wheelVec.x << ",  " << wheelVec.y << ") -> ";

        Angle angle = units::atan2(wheelVec.y, wheelVec.x);
        
        int dir = module.rotateTo(angle);
        
        std::cout << "(" << wheelVec.magnitude() * scale << ", " << angle << ") | ";
        module.move(wheelVec.magnitude() * scale * dir);
    }

    std::cout << "\n";
}
} // namespace libmavnetics

#include "robot/drive/swerveDrive.hpp"
#include "units/Angle.hpp"
#include "units/Vector2D.hpp"
#include "units/units.hpp"
#include <cstdint>

namespace libmavnetics {

Angle SwerveModule::getModuleAngle() {
  return units::constrainAngle360(rotateMotor.get_position() * 1_stDeg / rotateRatio);
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
    // Compute the maximum wheel speed to scale output if needed
    Number maxSpeed = 0;

    // Vector of wheel velocities
    std::vector<units::Vector2D<Number>> wheelVectors;

    for (const libmavnetics::SwerveModule& module : modules) {
        // Step 1: Calculate the rotational component (perpendicular to the locator vector)
        // Use z-rotation (cross product in 2D) for turn velocity
        units::Vector2D<Number> rotationVec = {
            -module.locator.y.convert(1_in) * trnVel,
             module.locator.x.convert(1_in) * trnVel
        };

        // Step 2: Combine translational and rotational vectors
        units::Vector2D<Number> wheelVec = {
            strVel + rotationVec.x,
            fwdVel + rotationVec.y
        };

        wheelVectors.push_back(wheelVec);

        // Track the max speed for normalization
        maxSpeed = std::max(maxSpeed, wheelVec.magnitude());
    }

    // Optional: normalize speeds if any exceed max motor power (127)
    constexpr Number maxMotorOutput = 127;
    Number scale = maxSpeed > maxMotorOutput ? (maxMotorOutput / maxSpeed) : 1;

    for (size_t i = 0; i < modules.size(); ++i) {
        libmavnetics::SwerveModule& module = modules[i];
        units::Vector2D<Number> wheelVec = wheelVectors[i];

        // Step 3: Get desired wheel angle
        Angle angle = units::atan2(wheelVec.y, wheelVec.x);

        // Step 4: Rotate to desired angle
        int dir = module.rotateTo(angle);

        // Step 5: Apply velocity with possible inversion
        module.move(wheelVec.magnitude() * scale * dir);
    }
}
} // namespace libmavnetics

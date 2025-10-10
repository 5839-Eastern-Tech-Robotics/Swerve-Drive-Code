#include "robot/drive/swerveDrive.hpp"
#include "units/Angle.hpp"
#include "units/Vector2D.hpp"
#include "units/units.hpp"
#include <algorithm>
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

// unused for now, just only correcting drift when the desired movement is low enough works well enough
AngularVelocity fromController(Number value) {
	return AngularVelocity(std::clamp(0.02852292 * value + 0.03589372, -0.646064256, 0.560643706));
}

SwerveDrive::SwerveDrive(std::vector<SwerveModule> modules, PID stabilityPID)
		: modules(modules), stabilityPID(stabilityPID) {}

void SwerveDrive::holonomic(Number fwdVel, Number strVel, Number trnVel) {

		Number maxSpeed = 0;
		std::vector<units::Vector2D<Number>> wheelVectors;

		for (const libmavnetics::SwerveModule& module : modules) {
				units::Vector2D<Number> rotationVec = {
						 -module.locator.y.convert(1_in) * trnVel,
						 -module.locator.x.convert(1_in) * trnVel
				};

				units::Vector2D<Number> wheelVec = {
						strVel + rotationVec.y,
						fwdVel + rotationVec.x
				};

				wheelVectors.push_back(wheelVec);

				maxSpeed = std::max(maxSpeed, wheelVec.magnitude());
		}

		Number scale = maxSpeed > 127 ? (127 / maxSpeed) : 1;
		
		for (size_t i = 0; i < modules.size(); ++i) {
				libmavnetics::SwerveModule& module = modules[i];
				units::Vector2D<Number> wheelVec = wheelVectors[i];

				Angle angle = units::atan2(wheelVec.y, wheelVec.x);
				
				int dir = module.rotateTo(angle);
				
				module.move(wheelVec.magnitude() * scale * dir);
		}

}

void SwerveDrive::driverControl(Angle heading, Number fwdVel, Number strVel, Number trnVel, bool absoluteControl) {
	if (trnVel > -30 && trnVel < 30) {
		trnVel += stabilityPID.update((prevHeading - heading).convert(1_stDeg));
	}

	if (absoluteControl) {
		float tmp = fwdVel * units::cos(heading) - strVel * units::sin(heading);
		strVel = fwdVel * units::sin(heading) + strVel * units::cos(heading);
		fwdVel = tmp;
	}

	holonomic(fwdVel, strVel, std::clamp(trnVel.convert(1), -128.0, 127.0));
}
} // namespace libmavnetics

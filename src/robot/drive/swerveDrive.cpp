#include "robot/drive/swerveDrive.hpp"
#include "globals.hpp"
#include "units/Angle.hpp"
#include "units/Vector2D.hpp"
#include "units/units.hpp"
#include <cstdint>

namespace libmavnetics {

Angle SwerveModule::getModuleAngle() {
	return 0_stDeg;
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

void SwerveModule::move(int8_t vel) { driveMotor.move(vel); }

void SwerveModule::move(LinearVelocity vel) {

}

void SwerveModule::moveVector(units::Vector2D<Number> vec) {
	int swap = rotateTo(vec.angleTo({0, 0}));
	move(vec.magnitude() * swap * 127);
}

SwerveDrive::SwerveDrive(std::vector<SwerveModule> modules) : modules(modules) {}

void SwerveDrive::holonomic(Number fwdVel, Number strVel, Number trnVel) {
				// save repeated calculations
				Number A = strVel - trnVel * driveLength / driveRadius;
				Number B = strVel + trnVel * driveLength / driveRadius;
				Number C = fwdVel + trnVel * driveWidth  / driveRadius;
				Number D = fwdVel - trnVel * driveWidth  / driveRadius;

				// calculate magnitude of drive vector
				Number FRSpeed = units::sqrt(B*B + C*C);
				Number FLSpeed = units::sqrt(B*B + D*D);
				Number BLSpeed = units::sqrt(A*A + D*D);
				Number BRSpeed = units::sqrt(A*A + C*C);

				// scale drive vectors to 0-1 range
				Number maxSpeed = std::max({FRSpeed, FLSpeed, BLSpeed, BRSpeed});
				if (maxSpeed > 1) {
						FRSpeed /= maxSpeed;
						FLSpeed /= maxSpeed;
						BLSpeed /= maxSpeed;
						BRSpeed /= maxSpeed;
				}

				// calculate heading for drive vector
				Angle FRAngle = units::atan2(B, C) / rotateRatio;
				Angle FLAngle = units::atan2(B, D) / rotateRatio;
				Angle BLAngle = units::atan2(A, D) / rotateRatio;
				Angle BRAngle = units::atan2(A, C) / rotateRatio;

				// calculate error to calculate minimum rotation later
				Angle FRError = FRAngle - units::remainder(from_cDeg(rotateFR.get_position()), (360 / rotateRatio));
				Angle FLError = FLAngle - units::remainder(from_cDeg(rotateFL.get_position()), (360 / rotateRatio));
				Angle BLError = BLAngle - units::remainder(from_cDeg(rotateBL.get_position()), (360 / rotateRatio));
				Angle BRError = BRAngle - units::remainder(from_cDeg(rotateBR.get_position()), (360 / rotateRatio));
				
				// Get smallest rotation algo (kinda skuffed lol)
				FRError = units::remainder(FRError, (360 / rotateRatio));
				FLError = units::remainder(FLError, (360 / rotateRatio));
				BLError = units::remainder(BLError, (360 / rotateRatio));
				BRError = units::remainder(BRError, (360 / rotateRatio));

				// Flip rotation and velocity direction
				if (FRError > from_stDeg(90) / rotateRatio)  { FRError -= from_stDeg(180) / rotateRatio; FRSpeed *= -1; }
				if (FRError < -from_stDeg(90) / rotateRatio) { FRError += from_stDeg(180) / rotateRatio; FRSpeed *= -1; }

				if (FLError > from_stDeg(90) / rotateRatio)  { FLError -= from_stDeg(180) / rotateRatio; FLSpeed *= -1; }
				if (FLError < -from_stDeg(90) / rotateRatio) { FLError += from_stDeg(180) / rotateRatio; FLSpeed *= -1; }

				if (BLError > from_stDeg(90) / rotateRatio)  { BLError -= from_stDeg(180) / rotateRatio; BLSpeed *= -1; }
				if (BLError < -from_stDeg(90) / rotateRatio) { BLError += from_stDeg(180) / rotateRatio; BLSpeed *= -1; }

				if (BRError > from_stDeg(90) / rotateRatio)  { BRError -= from_stDeg(180) / rotateRatio; BRSpeed *= -1; }
				if (BRError < -from_stDeg(90) / rotateRatio) { BRError += from_stDeg(180) / rotateRatio; BRSpeed *= -1; }

				// actually move to the angle using PID, TODO: retune PID
				rotateFR.move(FRPID.update(FRError.convert(1_stDeg)));
				rotateFL.move(FLPID.update(FLError.convert(1_stDeg)));
				rotateBL.move(BLPID.update(BLError.convert(1_stDeg)));
				rotateBR.move(BRPID.update(BRError.convert(1_stDeg)));

				// drive the wheels at the right velocity
				driveFR.move(FRSpeed * 127);
				driveFL.move(FLSpeed * 127);
				driveBL.move(BLSpeed * 127);
				driveBR.move(BRSpeed * 127);
}
} // namespace libmavnetics
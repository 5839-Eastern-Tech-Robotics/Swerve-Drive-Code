#include "libmavnetics/drive/odometry.hpp"

#include <cmath>
#include <iostream>
#include <numbers>

#include "libmavnetics/utils/pose2d.hpp"
#include "libmavnetics/utils/rotation2d.hpp"
#include "libmavnetics/utils/util.hpp"
#include "pros/adi.hpp"
#include "pros/imu.hpp"
#include "pros/rotation.hpp"
#include "pros/rtos.hpp"
#include "units/angle.h"
#include "units/length.h"
#include "units/math.h"
#include "units/time.h"

namespace libmavnetics {

OdometryModule::OdometryModule(pros::adi::Encoder *encoder,
                               units::meter_t wheelDiameter,
                               units::meter_t distance, double gearRatio)
    : encoder(encoder), diameter(wheelDiameter), distance(distance),
      gearRatio(gearRatio) {}

OdometryModule::OdometryModule(pros::Rotation *encoder,
                               units::meter_t wheelDiameter,
                               units::meter_t distance, double gearRatio)
    : rotation(rotation), diameter(wheelDiameter), distance(distance),
      gearRatio(gearRatio) {}

void OdometryModule::reset() {
  if (this->rotation != nullptr)
    this->rotation->reset();
  if (this->encoder != nullptr)
    this->encoder->reset();
}

units::meter_t OdometryModule::getDistanceTraveled() {
  if (this->rotation != nullptr)
    return this->rotation->get_position() / 360.0 * this->gearRatio *
           this->diameter * std::numbers::pi;
  if (this->encoder != nullptr)
    return this->encoder->get_value() / 36000.0 * this->gearRatio *
           this->diameter * std::numbers::pi;
  return 0_m;
}

units::meter_t OdometryModule::getOffset() { return this->distance; }

Odometry::Odometry(OdometryModule *vertical, OdometryModule *horizontal,
                   pros::Imu *imu)
    : vertical(vertical), horizontal(horizontal), imu(imu) {}

void calibrateIMU(pros::Imu *imu) {
  int attempt = 1;
  bool calibrated = false;
  // calibrate inertial, and if calibration fails, then repeat 5 times or until
  // successful
  while (attempt <= 5) {
    imu->reset();
    // wait until IMU is calibrated
    do
      pros::delay(10);
    while (imu->get_status() != pros::ImuStatus::error &&
           imu->is_calibrating());
    // exit if imu has been calibrated
    if (!std::isnan(imu->get_heading()) && !std::isinf(imu->get_heading())) {
      calibrated = true;
      break;
    }
    // indicate error
    pros::c::controller_rumble(pros::E_CONTROLLER_MASTER, "---");
    attempt++;
  }
  if (attempt > 5) {
    imu = nullptr;
  }
}

void Odometry::initUpdateLoop() {
  if (this->trackingTask == nullptr) {
    this->trackingTask = new pros::Task{[this] {
      while (true) {
        this->update();
        pros::delay(10);
      }
    }};
  }
}

bool Odometry::isTaskRunning() { return this->trackingTask != nullptr; }

void Odometry::calibrate(bool startTask, bool calibrateImu) {
  if (calibrateImu)
    calibrateIMU(this->imu);

  this->vertical->reset();
  this->horizontal->reset();

  if (startTask)
    initUpdateLoop();
}

void Odometry::resetPose() { this->setPose(0_m, 0_m, 0_deg); }

void Odometry::setPose(units::meter_t x, units::meter_t y,
                       units::degree_t theta) {
  setPose({x, y}, theta);
}

void Odometry::setPose(units::meter_t x, units::meter_t y, Rotation2D theta) {
  setPose({x, y}, theta);
}

void Odometry::setPose(Translation2D position, units::degree_t theta) {
  setPose(position, {theta});
}

void Odometry::setPose(Translation2D position, Rotation2D theta) {
  setPose({position, theta});
}

void Odometry::setPose(Pose2D pose) { this->pose = pose; }

Pose2D Odometry::getPose() { return this->pose; }

ChassisSpeeds Odometry::getLocalSpeed() { return this->localSpeed; }

ChassisSpeeds Odometry::getGlobalSpeed() { return this->globalSpeed; }

void Odometry::update() {
  units::meter_t verticalDist = vertical->getDistanceTraveled();
  units::meter_t horizontalDist = horizontal->getDistanceTraveled();
  units::degree_t imuAngle = units::degree_t{imu->get_rotation()};
  units::second_t time = pros::millis() * 1_ms;


  units::meter_t verticalOffset = vertical->getOffset();
  units::meter_t horizontalOffset = horizontal->getOffset();

  units::meter_t deltaVertical = verticalDist - prevVertical;
  units::meter_t deltaHorizontal = horizontalDist - prevHorizontal;
  units::degree_t deltaImu = imuAngle - prevIMU;
  units::second_t deltaTime = time - prevTime;

  prevVertical = verticalDist;
  prevHorizontal = horizontalDist;
  prevIMU = imuAngle;
  prevTime = time;

  units::radian_t heading = pose.rotation().radians();
  heading += deltaImu;
  units::radian_t deltaHeading = heading - pose.rotation().radians();
  units::radian_t averageHeading =
      pose.rotation().radians() + deltaHeading / 2.0;

  // std::cout << imuAngle.value() << ", " << deltaHeading.convert<units::degrees>().value() << std::endl;

  units::meter_t localX = 0_m;
  units::meter_t localY = 0_m;
  if (deltaHeading == 0_deg) {
    localX = deltaHorizontal;
    localY = deltaVertical;
  } else {
    localX = 2.0 * units::math::sin(deltaHeading / 2.0) *
             (deltaHorizontal / deltaHeading.value() + horizontalOffset);
    localY = 2.0 * units::math::sin(deltaHeading / 2.0) *
             (deltaVertical / deltaHeading.value() + verticalOffset);
  }

  // std::cout << "updating odometry, x: " << localX.value()
  //           << ", y: " << localY.value() << ", theta: " << deltaHeading.value()
  //           << std::endl;

  Pose2D prevPose = pose;
  pose = pose.transformBy({localY * units::math::sin(averageHeading) -
                        localX * units::math::cos(averageHeading),
                    localY * units::math::cos(averageHeading) +
                        localX * units::math::sin(averageHeading),
                    deltaHeading});

  // calculate local speed
  localSpeed.vx = ema(deltaHorizontal / deltaTime, localSpeed.vx, 0.95);
  localSpeed.vy = ema(deltaVertical / deltaTime, localSpeed.vy, 0.95);
  localSpeed.omega = ema(deltaHeading / deltaTime, localSpeed.omega, 0.95);

  // calculate global speed
  globalSpeed.vx =
      ema((pose.x() - prevPose.x()) / deltaTime, globalSpeed.vx, 0.95);
  globalSpeed.vy =
      ema((pose.y() - prevPose.y()) / deltaTime, globalSpeed.vy, 0.95);
  globalSpeed.omega = ema(deltaHeading / deltaTime, globalSpeed.omega, 0.95);
}

} // namespace libmavnetics

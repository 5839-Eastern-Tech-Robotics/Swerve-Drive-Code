#pragma once

#include "libmavnetics/utils/chassisSpeeds.hpp"
#include "libmavnetics/utils/pose2d.hpp"
#include "libmavnetics/utils/rotation2d.hpp"
#include "libmavnetics/utils/translation2d.hpp"
#include "libmavnetics/utils/chassisSpeeds.hpp"
#include "pros/adi.hpp"
#include "pros/imu.hpp"
#include "pros/motor_group.hpp"
#include "pros/rotation.hpp"
#include "pros/rtos.hpp"
#include "units/angle.h"
#include "units/angular_velocity.h"
#include "units/length.h"
#include "units/time.h"
#include "units/velocity.h"

namespace libmavnetics {

class OdometryModule {
public:
  OdometryModule(pros::adi::Encoder *encoder, units::meter_t wheelDiameter,
                 units::meter_t distance, double gearRatio = 1);
  OdometryModule(pros::Rotation *encoder, units::meter_t wheelDiameter,
                 units::meter_t distance, double gearRatio = 1);

  void reset();
  units::meter_t getDistanceTraveled();
  units::meters_per_second_t getVelocity();
  units::meter_t getOffset();

private:
  units::meter_t diameter;
  units::meter_t distance;
  units::revolutions_per_minute_t rpm;
  double gearRatio = 1;

  pros::adi::Encoder *encoder = nullptr;
  pros::Rotation *rotation = nullptr;
};

class Odometry {
public:
  Odometry(OdometryModule *vertical, OdometryModule *horizontal,
           pros::Imu *imu);

  void update();
  bool isTaskRunning();

  void calibrate(bool startTask = true, bool calibrateIMU = true);
  void resetPose();

  void setPose(units::meter_t x, units::meter_t y, units::degree_t theta);
  void setPose(units::meter_t x, units::meter_t y, Rotation2D theta);
  void setPose(Translation2D position, units::degree_t theta);
  void setPose(Translation2D position, Rotation2D theta);
  void setPose(Pose2D pose);

  Pose2D getPose();
  ChassisSpeeds getLocalSpeed();
  ChassisSpeeds getGlobalSpeed();
private:
  pros::Task *trackingTask = nullptr;

  Pose2D pose;
  ChassisSpeeds localSpeed;
  ChassisSpeeds globalSpeed;

  units::meter_t prevVertical = 0_m;
  units::meter_t prevHorizontal = 0_m;
  units::degree_t prevIMU = 0_deg;
  units::second_t prevTime = 0_s;

  OdometryModule *vertical;
  OdometryModule *horizontal;
  pros::Imu *imu;

  void initUpdateLoop();

};

} // namespace libmavnetics

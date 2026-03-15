#pragma once

#include <array>
#include <concepts>
#include <string>
#include <vector>

#include "liblvgl/misc/lv_types.h"
#include "libmavnetics/drive/swerveModule.hpp"
#include "libmavnetics/ui/core.h"
#include "libmavnetics/utils/chassisSpeeds.hpp"
#include "libmavnetics/utils/rotation2d.hpp"
#include "units/angular_velocity.h"
#include "units/velocity.h"

namespace libmavnetics {
namespace gui {

class SwerveVisualizer {
public:
  SwerveVisualizer(std::string name, units::meters_per_second_t maxModuleVelocity,
                   units::radians_per_second_t maxRotationalVelocity);

  SwerveVisualizer(units::meters_per_second_t maxModuleVelocity,
                   units::radians_per_second_t maxRotationalVelocity)
      : SwerveVisualizer("Swerve Visualizer", maxModuleVelocity,
                         maxRotationalVelocity) {}

  void setStates(ChassisSpeeds speed, std::array<SwerveModuleState, 4> states) {
    this->states = states;
    this->chassisSpeed = speed;
  }

  template <std::convertible_to<SwerveModuleState>... DriveState>
    requires(sizeof...(DriveState) == 4)
  void setStates(ChassisSpeeds speed, DriveState... states) {
    return setStates(speed, std::array<SwerveModuleState, 4>{states...});
  }

  void focus();

private:
  static std::array<lv_point_precise_t, 5>
  transformArrow(int length, Rotation2D angle);

  rd_view_t *view;
  std::string name;

  units::meters_per_second_t maxModuleVelocity;
  units::radians_per_second_t maxRotationalVelocity;

  std::array<SwerveModuleState, 4> states{};
  ChassisSpeeds chassisSpeed;

  lv_obj_t * arrow_fl;
  lv_obj_t * arrow_fr;
  lv_obj_t * arrow_bl;
  lv_obj_t * arrow_br;
  lv_obj_t * arrow_chassis;

  std::array<lv_point_precise_t, 5> arrow_points_fl;
  std::array<lv_point_precise_t, 5> arrow_points_fr;
  std::array<lv_point_precise_t, 5> arrow_points_bl;
  std::array<lv_point_precise_t, 5> arrow_points_br;
  std::array<lv_point_precise_t, 5> arrow_points_chassis;
  std::vector<lv_point_precise_t> chassis_frame{{0, 0}, {0, 150}, {150, 150}, {150, 0}, {0, 0}};
};

} // namespace gui
} // namespace libmavnetics

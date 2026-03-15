#include "autons.hpp"
#include "globals.hpp"
#include "pros/rtos.hpp"
#include "units/angular_velocity.h"

void red_left_side() {
    drivetrain.drive(maxLinearVelocity / 10, 0_mps, 0_rpm);
    pros::delay(3000);
    drivetrain.drive(0_mps, 0_mps, 0_rpm);
}

void red_right_side() {}

void blue_left_side() {}

void blue_right_side() {}

void skills() {}

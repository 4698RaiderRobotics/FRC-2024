#pragma once

#include <numbers>
#include <units/length.h>
#include <units/angle.h>
#include <units/velocity.h>
#include <units/acceleration.h>
#include <units/angular_velocity.h>
#include <units/angular_acceleration.h>

namespace pidf {
    constexpr double kShooterP = 0.0;
    constexpr double kShooterI = 0.0;
    constexpr double kShooterD = 0.0;

    constexpr double kShooterG = 0.0;
    constexpr double kShooterV = 0.0;
    constexpr double kShooterA = 0.0;
    constexpr double kShooterS = 0.0;

    constexpr double kShooterSpeed = 0.4;
}

namespace deviceIDs {
    constexpr int kTopShooterID = 14;
    constexpr int kBottomShooterID = 15;

    constexpr int kShooterEncoderID = 10;
}

namespace physical {
    // Maximum velocity and acceleration for the shooter angle TrapezoidProfile
    constexpr units::degrees_per_second_t kShooterMaxSpeed = 360_deg_per_s;
    constexpr units::degrees_per_second_squared_t kShooterMaxAcceleration = 540_deg_per_s_sq;

    // Minimum and maximum angles for the shooter position
    constexpr units::degree_t kShooterMaxAngle = 60_deg;
    constexpr units::degree_t kShooterMinAngle = 30_deg;

    constexpr double kShooterAbsoluteOffset = 0.0;
}
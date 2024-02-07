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

    constexpr double kArmP = 0.0;
    constexpr double kArmI = 0.0;
    constexpr double kArmD = 0.0;

    constexpr double kArmG = 0.0;
    constexpr double kArmV = 0.0;
    constexpr double kArmA = 0.0;
    constexpr double kArmS = 0.0;

    constexpr double kElevatorP = 0.0;
    constexpr double kElevatorI = 0.0;
    constexpr double kElevatorD = 0.0;

    constexpr double kElevatorG = 0.0;
    constexpr double kElevatorV = 0.0;
    constexpr double kElevatorA = 0.0;
    constexpr double kElevatorS = 0.0;
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

    // Maximum velocity and acceleration for the arm angle TrapezoidProfile
    constexpr units::degrees_per_second_t kArmMaxSpeed = 360_deg_per_s;
    constexpr units::degrees_per_second_squared_t kArmMaxAcceleration = 540_deg_per_s_sq;

    // Maximum velocity and acceleration for the elevator height TrapezoidProfile
    constexpr units::meters_per_second_t kElevatorMaxSpeed = 1_mps;
    constexpr units::meters_per_second_squared_t kElevatorMaxAcceleration = 1_mps_sq;

    // Minimum and maximum angles for the shooter position
    constexpr units::degree_t kShooterMaxAngle = 60_deg;
    constexpr units::degree_t kShooterMinAngle = 30_deg;

    // Minimum and maximum angles for the arm position
    constexpr units::degree_t kArmMaxAngle = 60_deg;
    constexpr units::degree_t kArmMinAngle = 30_deg;

    // Minimum and maximum heights for the elevator position
    constexpr units::meter_t kElevatorMaxHeight = 1_m;
    constexpr units::meter_t kElevatorMinHeight = 0_m;

    constexpr double kIntakeSpeed = 0.25;

    constexpr double kShooterAbsoluteOffset = 0.0;
}
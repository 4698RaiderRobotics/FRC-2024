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

    constexpr double kArmP = 0.0;
    constexpr double kArmI = 0.0;
    constexpr double kArmD = 0.0;

    constexpr double kArmG = 0.0;
    constexpr double kArmV = 0.0;
    constexpr double kArmA = 0.0;
    constexpr double kArmS = 0.0;

    constexpr double kWristP = 0.0;
    constexpr double kWristI = 0.0;
    constexpr double kWristD = 0.0;

    constexpr double kWristG = 0.0;
    constexpr double kWristV = 0.0;
    constexpr double kWristA = 0.0;
    constexpr double kWristS = 0.0;

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
    constexpr int kShooterAngleID = 16;

    constexpr int kIntakeID = 20;

    constexpr int kShooterEncoderID = 10;
    constexpr int kArmEncoderID = 18;
    constexpr int kWristEncoderID = 0;
}

namespace physical {
    // Maximum velocity for the shooter angle TrapezoidProfile
    constexpr units::degrees_per_second_t kShooterMaxSpeed = 360_deg_per_s;
    // Maximum acceleration for the shooter angle TrapezoidProfile
    constexpr units::degrees_per_second_squared_t kShooterMaxAcceleration = 540_deg_per_s_sq;

    // Maximum velocity for the arm angle TrapezoidProfile
    constexpr units::degrees_per_second_t kArmMaxSpeed = 360_deg_per_s;
    // Maximum acceleration for the arm angle TrapezoidProfile
    constexpr units::degrees_per_second_squared_t kArmMaxAcceleration = 540_deg_per_s_sq;

    // Maximum velocity for the wrist angle TrapezoidProfile
    constexpr units::degrees_per_second_t kWristMaxSpeed = 360_deg_per_s;
    // Maximum acceleration for the wrist angle TrapezoidProfile
    constexpr units::degrees_per_second_squared_t kWristMaxAcceleration = 540_deg_per_s_sq;

    // Maximum velocity for the elevator height TrapezoidProfile
    constexpr units::meters_per_second_t kElevatorMaxSpeed = 1_mps;
    // Maximum acceleration for the elevator height TrapezoidProfile
    constexpr units::meters_per_second_squared_t kElevatorMaxAcceleration = 1_mps_sq;

    // Maximum angle for the shooter position
    constexpr units::degree_t kShooterMaxAngle = 70_deg;
    // Minimum angle for the shooter position
    constexpr units::degree_t kShooterMinAngle = 10_deg;

    // Maximum angle for the arm position
    constexpr units::degree_t kArmMaxAngle = 60_deg;
    // Minimum angle for the arm position
    constexpr units::degree_t kArmMinAngle = 30_deg;

    // Maximum angle for the wrist position
    constexpr units::degree_t kWristMaxAngle = 60_deg;
    // Minimum angle for the wrist position
    constexpr units::degree_t kWristMinAngle = 30_deg;

    // Maximum height for the elevator position
    constexpr units::meter_t kElevatorMaxHeight = 1_m;
    // Minimum height for the elevator position
    constexpr units::meter_t kElevatorMinHeight = 0_m;

    constexpr double kIntakeSpeed = 0.25;

    constexpr double kShooterSpeed = 0.4;

    // Angle for the wrist to pick up off the ground
    constexpr units::degree_t kWristGroundPickUpAngle = 0_deg;
    // Angle for the arm to pick up off the ground
    constexpr units::degree_t kArmGroundPickUpAngle = 0_deg;

    // Angle for the wrist to rest at
    constexpr units::degree_t kWristPassiveAngle = 0_deg;
    // Angle for the arm to rest at
    constexpr units::degree_t kArmPassiveAngle = 0_deg;

    // Angle for the wrist to shoot from
    constexpr units::degree_t kWristShootingAngle = 0_deg;
    // Angle for the arm to shoot from
    constexpr units::degree_t kArmShootingAngle = 0_deg;

    // Angle for the wrist to place in amp
    constexpr units::degree_t kWristAmpAngle = 0_deg;
    // Angle for the arm to place in amp
    constexpr units::degree_t kArmAmpAngle = 0_deg;

    // Height for the elevator to place in amp
    constexpr units::meter_t kElevatorAmpHeight = 0_m;

    constexpr double kShooterAbsoluteOffset = 0.0;

    constexpr units::second_t kDt = 20_ms;
}
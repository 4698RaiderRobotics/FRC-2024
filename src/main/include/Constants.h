#pragma once

#include <numbers>
#include <units/length.h>
#include <units/angle.h>
#include <units/velocity.h>
#include <units/acceleration.h>
#include <units/angular_velocity.h>
#include <units/angular_acceleration.h>

namespace pidf {
    constexpr double kShooterP = 0.02;
    constexpr double kShooterI = 0.0;
    constexpr double kShooterD = 0.0;

    constexpr double kShooterG = 0.1;
    constexpr double kShooterV = 8.0;
    constexpr double kShooterA = 0.0;
    constexpr double kShooterS = 0.0;

    constexpr double kArmP = 0.001;
    constexpr double kArmI = 0.0;
    constexpr double kArmD = 0.0;

    constexpr double kArmG = 0.2;
    constexpr double kArmWristG = 0.07;
    constexpr double kArmV = 1.1;
    constexpr double kArmA = 0.0;
    constexpr double kArmS = 0.0;

    constexpr double kWristP = 1.0;
    constexpr double kWristI = 0.0;
    constexpr double kWristD = 0.0;

    constexpr double kWristG = 0.0115;
    constexpr double kWristV = 0.65;
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

    constexpr int kArmMotorID = 21;
    constexpr int kWristMotorID = 22;

    constexpr int kIntakeID = 20;

    constexpr int kShooterEncoderID = 17;
    constexpr int kArmEncoderID = 23;
    constexpr int kWristEncoderID = 24;
}

namespace physical {
    // Maximum velocity for the shooter angle TrapezoidProfile
    constexpr units::degrees_per_second_t kShooterMaxSpeed = 360_deg_per_s;
    // Maximum acceleration for the shooter angle TrapezoidProfile
    constexpr units::degrees_per_second_squared_t kShooterMaxAcceleration = 360_deg_per_s_sq;

    // Maximum velocity for the arm angle TrapezoidProfile
    constexpr units::degrees_per_second_t kArmMaxSpeed = 360_deg_per_s;
    // Maximum acceleration for the arm angle TrapezoidProfile
    constexpr units::degrees_per_second_squared_t kArmMaxAcceleration = 360_deg_per_s_sq;

    // Maximum velocity for the wrist MotionMagic profile
    // Units in rotations per second
    constexpr double kWristMaxSpeed = 0.5;
    // Maximum acceleration for the wrist MotionMagic profile
    // Units in rotations per second squared
    constexpr double kWristMaxAcceleration = 1;

    // Maximum velocity for the elevator height TrapezoidProfile
    constexpr units::meters_per_second_t kElevatorMaxSpeed = 1_mps;
    // Maximum acceleration for the elevator height TrapezoidProfile
    constexpr units::meters_per_second_squared_t kElevatorMaxAcceleration = 1_mps_sq;

    // Maximum angle for the shooter position
    constexpr units::degree_t kShooterMaxAngle = 75_deg;
    // Minimum angle for the shooter position
    constexpr units::degree_t kShooterMinAngle = 15_deg;

    // Maximum angle for the arm position
    constexpr units::degree_t kArmMaxAngle = 160_deg;
    // Minimum angle for the arm position
    constexpr units::degree_t kArmMinAngle = 0_deg;

    // Maximum angle for the wrist position
    constexpr units::degree_t kWristMaxAngle = 180_deg;
    // Minimum angle for the wrist position
    constexpr units::degree_t kWristMinAngle = -90_deg;

    // Maximum height for the elevator position
    constexpr units::meter_t kElevatorMaxHeight = 1_m;
    // Minimum height for the elevator position
    constexpr units::meter_t kElevatorMinHeight = 0_m;

    constexpr double kIntakeSpeed = 0.25;

    constexpr double kShooterSpeed = 0.25;

    // Angle for the shooter to shoot from speaker
    constexpr units::degree_t kShooterSpeakerAngle = 60_deg;
    // Angle for the shooter to shoot from podium
    constexpr units::degree_t kShooterPodiumAngle = 30_deg;

    // Angle for the wrist to pick up off the ground
    constexpr units::degree_t kWristGroundPickUpAngle = -60_deg;
    // Angle for the arm to pick up off the ground
    constexpr units::degree_t kArmGroundPickUpAngle = 10_deg;

    // Angle for the wrist to rest at
    constexpr units::degree_t kWristPassiveAngle = 0_deg;
    // Angle for the arm to rest at
    constexpr units::degree_t kArmPassiveAngle = 135_deg;

    // Angle for the wrist to shoot from
    constexpr units::degree_t kWristShootingAngle = 160_deg;
    // Angle for the arm to shoot from
    constexpr units::degree_t kArmShootingAngle = 135_deg;

    // Angle for the wrist to place in amp
    constexpr units::degree_t kWristAmpAngle = 0_deg;
    // Angle for the arm to place in amp
    constexpr units::degree_t kArmAmpAngle = 0_deg;

    // Height for the elevator to place in amp
    constexpr units::meter_t kElevatorAmpHeight = 0_m;

    // Height for the elevator to place in trap
    constexpr units::meter_t kElevatorTrapHeight = 0_m;

    constexpr double kShooterAbsoluteOffset = 0.301;
    constexpr double kWristAbsoluteOffset = 0.145;
    constexpr double kArmAbsoluteOffset = 0.081;

    constexpr units::second_t kDt = 20_ms;
}
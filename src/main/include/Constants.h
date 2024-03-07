#pragma once

#include <numbers>
#include <units/length.h>
#include <units/angle.h>
#include <units/velocity.h>
#include <units/acceleration.h>
#include <units/angular_velocity.h>
#include <units/angular_acceleration.h>
#include <frc/geometry/Transform3d.h>

namespace pidf {
    constexpr double kShooterP = 0.02;
    constexpr double kShooterI = 0.0;
    constexpr double kShooterD = 0.0;

    constexpr double kShooterG = 0.1;
    constexpr double kShooterV = 8.0;
    constexpr double kShooterA = 0.0;
    constexpr double kShooterS = 0.0;

    constexpr double kArmP = 0.002;
    constexpr double kArmI = 0.0;
    constexpr double kArmD = 0.0;

    constexpr double kArmG = 0.15;
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

    constexpr double kElevatorP = 6.666;
    constexpr double kElevatorI = 0.0;
    constexpr double kElevatorD = 0.0;

    constexpr double kElevatorG = 0.75;
    constexpr double kElevatorV = 3.333;
    constexpr double kElevatorA = 0.0;
    constexpr double kElevatorS = 0.0;

    constexpr double kSpeedP = 0.001;
    constexpr double kSpeedI = 0.0;
    constexpr double kSpeedD = 0.0;
    constexpr double kSpeedFF = 0.00018;
}

namespace deviceIDs {
    constexpr int kTopShooterID = 14;
    constexpr int kBottomShooterID = 15;
    constexpr int kShooterAngleID = 16;

    constexpr int kArmMotorID = 21;
    constexpr int kWristMotorID = 22;

    constexpr int kElevatorID = 18;

    constexpr int kClimberID = 19;

    constexpr int kIntakeID = 20;

    constexpr int kShooterEncoderID = 17;
    constexpr int kArmEncoderID = 23;
    constexpr int kWristEncoderID = 24;
}

namespace physical {

    const frc::Transform3d kRobotToCam = frc::Transform3d(frc::Translation3d(-0.5_m, 0_m, 0.25_m), frc::Rotation3d(0_deg, -35_deg, 180_deg));

    // Maximum velocity for the shooter angle TrapezoidProfile
    constexpr units::degrees_per_second_t kShooterMaxSpeed = 360_deg_per_s;
    // Maximum acceleration for the shooter angle TrapezoidProfile
    constexpr units::degrees_per_second_squared_t kShooterMaxAcceleration = 360_deg_per_s_sq;

    // Maximum velocity for the arm angle TrapezoidProfile
    constexpr units::degrees_per_second_t kArmMaxSpeed = 540_deg_per_s;
    // Maximum acceleration for the arm angle TrapezoidProfile
    constexpr units::degrees_per_second_squared_t kArmMaxAcceleration = 720_deg_per_s_sq;

    // Maximum velocity for the wrist MotionMagic profile
    // Units in rotations per second
    constexpr double kWristMaxSpeed = 1.5;
    // Maximum acceleration for the wrist MotionMagic profile
    // Units in rotations per second squared
    constexpr double kWristMaxAcceleration = 3;

    // Maximum velocity for the elevator height TrapezoidProfile
    constexpr units::meters_per_second_t kElevatorMaxSpeed = 2_mps;
    // Maximum acceleration for the elevator height TrapezoidProfile
    constexpr units::meters_per_second_squared_t kElevatorMaxAcceleration = 4_mps_sq;

    // Maximum angle for the shooter position
    constexpr units::degree_t kShooterMaxAngle = 75_deg;
    // Minimum angle for the shooter position
    constexpr units::degree_t kShooterMinAngle = 15_deg;

    // Maximum angle for the arm position
    constexpr units::degree_t kArmMaxAngle = 180_deg;
    // Minimum angle for the arm position
    constexpr units::degree_t kArmMinAngle = -8_deg;

    // Maximum angle for the wrist position
    constexpr units::degree_t kWristMaxAngle = 180_deg;
    // Minimum angle for the wrist position
    constexpr units::degree_t kWristMinAngle = -90_deg;

    // Maximum height for the elevator position
    constexpr units::meter_t kElevatorMaxHeight = 27_in;
    // Minimum height for the elevator position
    constexpr units::meter_t kElevatorMinHeight = 0_in;

    constexpr double kIntakeSpeed = 1;

    constexpr units::revolutions_per_minute_t kShooterSpeed = 3000_rpm;

    // Angle for the shooter to shoot from speaker
    constexpr units::degree_t kShooterSpeakerAngle = 60_deg;
    // Angle for the shooter to shoot from podium
    constexpr units::degree_t kShooterPodiumAngle = 22_deg;

    // Angle for the wrist to pick up off the ground
    constexpr units::degree_t kWristGroundPickUpAngle = -30_deg;
    // Angle for the arm to pick up off the ground
    constexpr units::degree_t kArmGroundPickUpAngle = -8_deg;

    // Angle for the wrist to rest at
    constexpr units::degree_t kWristPassiveAngle = 30_deg;
    // Angle for the arm to rest at
    constexpr units::degree_t kArmPassiveAngle = 170_deg;

    // Angle for the wrist to shoot from
    constexpr units::degree_t kWristShootingAngle = 140_deg;
    // Angle for the arm to shoot from
    constexpr units::degree_t kArmShootingAngle = 170_deg;

    // Angle for the wrist to place in amp
    constexpr units::degree_t kWristAmpAngle = 90_deg;
    // Angle for the arm to place in amp
    constexpr units::degree_t kArmAmpAngle = 45_deg;

    // Height for the elevator to place in amp
    constexpr units::meter_t kElevatorAmpHeight = 10_in;

    // Height for the elevator to place in trap
    constexpr units::meter_t kElevatorTrapHeight = 0_m;

    constexpr double kShooterAbsoluteOffset = -0.342;
    constexpr double kWristAbsoluteOffset = 0.206;
    constexpr double kArmAbsoluteOffset = 0.145;

    constexpr units::second_t kDt = 20_ms;
}
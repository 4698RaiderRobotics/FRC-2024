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

    constexpr double kArmP = 0.006;
    constexpr double kArmI = 0.0;
    constexpr double kArmD = 0.0;

    constexpr double kArmG = 0.3;
    constexpr double kArmWristG = 0.01;
    constexpr double kArmV = 1.1;
    constexpr double kArmA = 0.0;
    constexpr double kArmS = 0.0;

    constexpr double kWristP = 1.0;
    constexpr double kWristI = 0.0;
    constexpr double kWristD = 0.0;

    constexpr double kWristG = 0.018;
    constexpr double kWristV = 0.38;
    constexpr double kWristA = 0.0;
    constexpr double kWristS = 0.005;

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
    constexpr int kRightShooterID = 14;
    constexpr int kLeftShooterID = 15;
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

        // Camera "CameraA_1MP"
    // const frc::Transform3d kFrontRightRobotToCam = frc::Transform3d(frc::Translation3d{0.025_m, -0.26_m, 0.66_m}, frc::Rotation3d{-6_deg, -8_deg, 30_deg});
    const frc::Transform3d kFrontRightRobotToCam = 
            frc::Transform3d(frc::Translation3d{-1.9015_in, -10.8912_in, 30.2843_in}, frc::Rotation3d{-8.2028_deg, -23.9351_deg, 36.3362_deg});


         // Camera "CameraB_2MP"
//    const frc::Transform3d kBackRightRobotToCam = frc::Transform3d(frc::Translation3d{-0.06_m, -0.26_m, 0.66_m}, frc::Rotation3d{10_deg, -15_deg, 150_deg});
   const frc::Transform3d kBackRightRobotToCam = 
            frc::Transform3d(frc::Translation3d{-3.3483_in, -10.3919_in, 26.2791_in}, frc::Rotation3d{9.487_deg, -14.549_deg, 148.395_deg});

    // const frc::Transform3d kFrontLeftRobotToCam = frc::Transform3d(frc::Translation3d(0_m, 0_m, 0_m), frc::Rotation3d(0_deg, 0_deg, 0_deg));

         // Camera "CameraC_2MP"
    // const frc::Transform3d kBackLeftRobotToCam = frc::Transform3d(frc::Translation3d(-0.06_m, 0.26_m, 0.66_m), frc::Rotation3d(-8_deg, -14_deg, 207_deg));
    const frc::Transform3d kBackLeftRobotToCam = 
            frc::Transform3d(frc::Translation3d(-6.7061_in, 9.7004_in, 24.6382_in), frc::Rotation3d(-8.760_deg, -13.443_deg, -149.618_deg));


    // *****************     SHOOTER SUBSYSTEM      **********************

    // Maximum velocity for the shooter angle TrapezoidProfile
    constexpr units::degrees_per_second_t kShooterMaxSpeed = 360_deg_per_s;
    // Maximum acceleration for the shooter angle TrapezoidProfile
    constexpr units::degrees_per_second_squared_t kShooterMaxAcceleration = 360_deg_per_s_sq;

    // Maximum angle for the shooter position
    constexpr units::degree_t kShooterMaxAngle = 75_deg;
    // Minimum angle for the shooter position
    constexpr units::degree_t kShooterMinAngle = 0_deg;

    // Angle for the shooter to shoot from speaker
    constexpr units::degree_t kShooterSpeakerAngle = 60_deg;
    // Angle for the shooter to shoot from podium
    constexpr units::degree_t kShooterPodiumAngle = 22_deg;



    // *****************     ARM SUBSYSTEM      **********************

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

    // Maximum angle for the arm position
    constexpr units::degree_t kArmMaxAngle = 180_deg;
    // Minimum angle for the arm position
    constexpr units::degree_t kArmMinAngle = -15_deg;

    // Maximum angle for the wrist position
    constexpr units::degree_t kWristMaxAngle = 180_deg;
    // Minimum angle for the wrist position
    constexpr units::degree_t kWristMinAngle = -90_deg;

    // Angle for the wrist to pick up off the ground
    constexpr units::degree_t kWristGroundPickUpAngle = -45_deg;
    // Angle for the arm to pick up off the ground
    constexpr units::degree_t kArmGroundPickUpAngle = -15_deg;

    // Angle for the wrist to rest at
    constexpr units::degree_t kWristPassiveAngle = 40_deg;
    // Angle for the arm to rest at
    constexpr units::degree_t kArmPassiveAngle = 155_deg;

    // Angle for the wrist to place in amp
    constexpr units::degree_t kWristAmpAngle = 90_deg;
    // Angle for the arm to place in amp
    constexpr units::degree_t kArmAmpAngle = 60_deg;
    // Angle for the wrist to spit note in amp
    constexpr units::degree_t kWristAmpSpitAngle = 112_deg;
     // Angle for the wrist to drop down from amp
    constexpr units::degree_t kWristAmpDropAngle = 80_deg;
    // Angle for the arm to drop down from amp
    constexpr units::degree_t kArmAmpDropAngle = 70_deg;


    // *****************     ELEVATOR SUBSYSTEM      **********************

    // Maximum velocity for the elevator height TrapezoidProfile
    constexpr units::meters_per_second_t kElevatorMaxSpeed = 2_mps;
    // Maximum acceleration for the elevator height TrapezoidProfile
    constexpr units::meters_per_second_squared_t kElevatorMaxAcceleration = 4_mps_sq;

    // Maximum height for the elevator position
    constexpr units::inch_t kElevatorMaxHeight = 27_in;
    // Minimum height for the elevator position
    constexpr units::inch_t kElevatorMinHeight = 0_in;

    // Height for the elevator to place in amp
    constexpr units::inch_t kElevatorAmpHeight = 19_in;

    // Height for the elevator to place in trap
    constexpr units::inch_t kElevatorTrapHeight = 25_in;


    // *****************     CLIMBER SUBSYSTEM      **********************

    // Maximum height for the climber position
    constexpr units::inch_t kClimberMaxHeight = 18.5_in;
    // Minimum height for the climber position
    constexpr units::inch_t kClimberMinHeight = 0_in;
    // Resting height for the climber 
    constexpr units::inch_t kClimberRestHeight = 13_in;

    // Intermediate height for the auto climb and trap command
    constexpr units::inch_t kClimberMidHeight = 7_in;

    constexpr units::second_t kDt = 20_ms;
}
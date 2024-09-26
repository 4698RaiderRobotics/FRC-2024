#pragma once

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


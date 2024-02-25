// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc2/command/SubsystemBase.h>

#include <wpi/array.h>
#include <wpi/DataLog.h>

#include <frc/kinematics/SwerveDriveKinematics.h>
#include <frc/estimator/SwerveDrivePoseEstimator.h>
#include <frc/kinematics/ChassisSpeeds.h>
#include <frc/controller/HolonomicDriveController.h>
#include <ctre/phoenix/sensors/PigeonIMU.h>
#include <ctre/phoenix6/Pigeon2.hpp>
#include <frc/geometry/Pose2d.h>
#include <units/time.h>
using namespace units::literals;

#include "SwerveModule.h"
#include "SwerveModuleDisplay.h"
#include "subsystems/VisionSubsystem.h"

class SwerveDriveSubsystem : public frc2::SubsystemBase {
 public:
  
  // Components (e.g. motor controllers and sensors) should generally be
  // declared private and exposed only through public methods.
  SwerveDriveSubsystem(VisionSubsystem *ll);
    
    void ArcadeDrive( double xPercent, double yPercent, double omegaPercent, bool fieldRelative = true );
    void Drive( frc::ChassisSpeeds speeds, bool fieldRelative = true );

    void DriveTrajectory( frc::Trajectory::State trajState, const frc::Rotation2d &robotHeading );

    void Periodic( void );

    frc::Pose2d GetPose( void );

    void ResetGyro( units::degree_t angle );

    void ResetPose( frc::Translation2d position );

    void StartLogging( wpi::log::DataLog& log );

  private:
    void LogSwerveStateArray(  wpi::log::DoubleArrayLogEntry& logEntry, wpi::array<frc::SwerveModuleState, 4U> states );
    void TuneSwerveDrive();

  
    VisionSubsystem *m_vision;

    SwerveModule m_modules[4];

    frc::Trajectory m_trajectory;
    wpi::array<frc::SwerveModuleState, 4U> m_desiredStates{ wpi::empty_array };
    wpi::array<frc::SwerveModuleState, 4U> m_actualStates{ wpi::empty_array };


    ctre::phoenix6::hardware::Pigeon2 m_gyro;

    frc::Translation2d m_mod_Location[4];

    frc::SwerveDriveKinematics<4> m_kinematics;
    frc::SwerveDrivePoseEstimator<4> m_odometry;

    // Drive controller for driving a trajectory
    frc::HolonomicDriveController m_controller;


    wpi::log::DoubleArrayLogEntry m_actualLogEntry;
    wpi::log::DoubleArrayLogEntry m_desiredLogEntry;
    wpi::log::DoubleArrayLogEntry m_poseLogEntry;
    wpi::log::DoubleLogEntry m_gyroYawLogEntry;
    bool m_logging{ false };
};

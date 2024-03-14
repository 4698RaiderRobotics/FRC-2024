// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc/geometry/Pose2d.h>
#include <frc2/command/CommandHelper.h>
#include <frc2/command/SequentialCommandGroup.h>

#include "LUT.h"
#include "ControllerAxis.h"

#include "LoggedCommand.h"
#include "subsystems/SwerveDriveSubsystem.h"
#include "subsystems/IntakeSubsystem.h"
#include "subsystems/ShooterSubsystem.h"
#include "subsystems/ArmSubsystem.h"
#include "subsystems/VisionSubsystem.h"
#include "subsystems/ElevatorSubsystem.h"

class ShootNoteTargeting
    : public frc2::CommandHelper<LoggedCommand, ShootNoteTargeting> {
public:
  ShootNoteTargeting( SwerveDriveSubsystem* swerve, ShooterSubsystem* shooter, IntakeSubsystem* intake, 
                      ArmSubsystem* arm, ElevatorSubsystem* elev, VisionSubsystem* vision, 
                      ControllerAxis *x_axis=nullptr, ControllerAxis *y_axis=nullptr );

  void Init() override;

  void Execute() override;

  void HasEnded(bool interrupted) override;

  bool IsFinished() override;

private:
    SwerveDriveSubsystem* m_drive;
    ShooterSubsystem* m_shooter;
    IntakeSubsystem* m_intake;
    ArmSubsystem* m_arm;
    ElevatorSubsystem* m_elev;
    VisionSubsystem* m_vision;

    ControllerAxis *m_x_axis;
    ControllerAxis *m_y_axis;

    bool readyToShoot;
    bool noTargets;
    bool allowDriving;

    frc::Pose2d targetLocation;

    units::second_t spin_start;

      // NOTE: Should change to Geometry based calculation.
    // LUT pitchToAngle{ {-17.0, -11.7, -7.29, 12.3}, {18.0, 22.0, 25.0, 45.0}};
    LUT pitchToAngle{ {-17.0, -11.7, -7.29, 12.3}, {20.0, 24.0, 27.0, 45.0}};
    // LUT pitchToAngle{ {-17.0, -11.7, -7.29, 12.3}, {22.0, 26.0, 29.0, 45.0}};
};
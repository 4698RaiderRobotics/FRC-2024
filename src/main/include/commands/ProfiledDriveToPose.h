// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc2/command/Command.h>
#include <frc2/command/CommandHelper.h>

#include "LoggedCommand.h"
#include "subsystems/SwerveDriveSubsystem.h"
#include "subsystems/VisionSubsystem.h"

/**
 * An example command.
 *
 * <p>Note that this extends CommandHelper, rather extending Command
 * directly; this is crucially important, or else the decorator functions in
 * Command will *not* work!
 */
class ProfiledDriveToPose
    : public frc2::CommandHelper<LoggedCommand, ProfiledDriveToPose> {
 public:
  ProfiledDriveToPose(SwerveDriveSubsystem *swerve, VisionSubsystem *vision, frc::Pose2d targetPose);

  void Init() override;

  void Execute() override;

  void Ending(bool interrupted) override;

  bool IsFinished() override;

 private:
  SwerveDriveSubsystem *m_swerve;
  VisionSubsystem *m_vision;

  frc::Pose2d m_targetPose;

  frc::TrapezoidProfile<units::meters> m_xProfile{{2_mps, 1_mps_sq}};
  frc::TrapezoidProfile<units::meters> m_yProfile{{2_mps, 1_mps_sq}};
  frc::TrapezoidProfile<units::degrees> m_omegaProfile{{180_deg_per_s, 180_deg_per_s_sq}};

  frc::TrapezoidProfile<units::meters>::State m_xSetpoint;
  frc::TrapezoidProfile<units::meters>::State m_ySetpoint;
  frc::TrapezoidProfile<units::degrees>::State m_omegaSetpoint;

  frc::TrapezoidProfile<units::meters>::State m_xGoal;
  frc::TrapezoidProfile<units::meters>::State m_yGoal;
  frc::TrapezoidProfile<units::degrees>::State m_omegaGoal;

  units::second_t m_startTime;
};

// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc2/command/Command.h>
#include <frc2/command/CommandHelper.h>

#include "subsystems/SwerveDriveSubsystem.h"

/**
 * An example command.
 *
 * <p>Note that this extends CommandHelper, rather extending Command
 * directly; this is crucially important, or else the decorator functions in
 * Command will *not* work!
 */
class FollowTrajectory
    : public frc2::CommandHelper<frc2::Command, FollowTrajectory> {
 public:
  FollowTrajectory(SwerveDriveSubsystem *swerve, frc::Trajectory trajectory, frc::Rotation2d endAngle);

  void Initialize() override;

  void Execute() override;

  void End(bool interrupted) override;

  bool IsFinished() override;
 private:
  SwerveDriveSubsystem *m_swerve;
  frc::Trajectory m_trajectory;
  frc::Rotation2d m_endAngle;

  units::second_t m_autoElapsed;

  bool atPosition;
};

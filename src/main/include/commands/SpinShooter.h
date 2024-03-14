// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc2/command/Command.h>
#include <frc2/command/CommandHelper.h>

#include "LoggedCommand.h"
#include "subsystems/ShooterSubsystem.h"

/**
 * An example command.
 *
 * <p>Note that this extends CommandHelper, rather extending Command
 * directly; this is crucially important, or else the decorator functions in
 * Command will *not* work!
 */
class SpinShooter
    : public frc2::CommandHelper<LoggedCommand, SpinShooter> {
 public:
  SpinShooter(ShooterSubsystem* shooter, units::revolutions_per_minute_t speed);

  void Init() override;

  void Execute() override;

  void HasEnded(bool interrupted) override;

  bool IsFinished() override;
 private:
  ShooterSubsystem* m_shooter;
  units::revolutions_per_minute_t m_speed;
  units::second_t m_startTime;
};

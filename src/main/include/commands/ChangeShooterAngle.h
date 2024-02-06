// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc2/command/Command.h>
#include <frc2/command/CommandHelper.h>

#include "subsystems/ShooterSubsystem.h"

/**
 * An example command.
 *
 * <p>Note that this extends CommandHelper, rather extending Command
 * directly; this is crucially important, or else the decorator functions in
 * Command will *not* work!
 */
// Updates the setpoint for the shooter's angle
class ChangeShooterAngle
    : public frc2::CommandHelper<frc2::Command, ChangeShooterAngle> {
 public:
  ChangeShooterAngle(ShooterSubsystem* shooter, units::degree_t angle);

  void Initialize() override;

  void Execute() override;

  void End(bool interrupted) override;

  bool IsFinished() override;
 private:
  ShooterSubsystem* m_shooter;
  units::degree_t m_angle;
};

// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc2/command/Command.h>
#include <frc2/command/CommandHelper.h>

#include "LoggedCommand.h"
#include "subsystems/ClimberSubsystem.h"

/**
 * An example command.
 *
 * <p>Note that this extends CommandHelper, rather extending Command
 * directly; this is crucially important, or else the decorator functions in
 * Command will *not* work!
 */
class Climb
    : public frc2::CommandHelper<LoggedCommand, Climb> {
 public:
  Climb(ClimberSubsystem *climber);

  void Init() override;

  void Execute() override;

  void Ending(bool interrupted) override;

  bool IsFinished() override;

 private:
  ClimberSubsystem *m_climber;
};

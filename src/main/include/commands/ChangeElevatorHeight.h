// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc2/command/Command.h>
#include <frc2/command/CommandHelper.h>

#include "LoggedCommand.h"
#include "subsystems/ElevatorSubsystem.h"

/**
 * An example command.
 *
 * <p>Note that this extends CommandHelper, rather extending Command
 * directly; this is crucially important, or else the decorator functions in
 * Command will *not* work!
 */
class ChangeElevatorHeight
    : public frc2::CommandHelper<LoggedCommand, ChangeElevatorHeight> {
 public:
  ChangeElevatorHeight(ElevatorSubsystem* elevator, units::meter_t height);

  void Init() override;

  void Execute() override;

  void HasEnded(bool interrupted) override;

  bool IsFinished() override;

 private:
  ElevatorSubsystem* m_elevator;
  units::meter_t m_height;
};

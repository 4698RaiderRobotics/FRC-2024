// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc2/command/Command.h>
#include <frc2/command/CommandHelper.h>

#include "LoggedCommand.h"

class IntakeSubsystem;

/**
 * An example command.
 *
 * <p>Note that this extends CommandHelper, rather extending Command
 * directly; this is crucially important, or else the decorator functions in
 * Command will *not* work!
 */
class IntakeNote
    : public frc2::CommandHelper<LoggedCommand, IntakeNote> {
 public:
  IntakeNote(IntakeSubsystem* intake);

  void Init() override;

  void Execute() override;

  void Ending(bool interrupted) override;

  bool IsFinished() override;
 private:
  IntakeSubsystem* m_intake;
  LEDSubsystem* m_leds;

  units::second_t m_startTime;
  // double startPos;
  bool beamHasBroken = false;
  // bool isFinished = false;
};

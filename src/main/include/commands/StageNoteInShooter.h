// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc2/command/CommandHelper.h>

#include "LoggedCommand.h"

class IntakeSubsystem;
class ShooterSubsystem;
class ArmSubsystem;
class ElevatorSubsystem;

class StageNoteInShooter
    : public frc2::CommandHelper<LoggedCommand, StageNoteInShooter> {
public:
  StageNoteInShooter( ShooterSubsystem* shooter, IntakeSubsystem* intake, 
                      ArmSubsystem* arm, ElevatorSubsystem* elev );

  void Init() override;

  void Execute() override;

  void Ending(bool interrupted) override;

  bool IsFinished() override;

private:
    ShooterSubsystem* m_shooter;
    IntakeSubsystem* m_intake;
    ArmSubsystem* m_arm;
    ElevatorSubsystem* m_elev;
};
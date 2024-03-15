// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

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
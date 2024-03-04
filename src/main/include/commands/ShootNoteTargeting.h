// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc2/command/CommandHelper.h>
#include <frc2/command/SequentialCommandGroup.h>

#include "subsystems/IntakeSubsystem.h"
#include "subsystems/ShooterSubsystem.h"
#include "subsystems/ArmSubsystem.h"
#include "subsystems/VisionSubsystem.h"

class ShootNoteTargeting
    : public frc2::CommandHelper<frc2::Command, ShootNoteTargeting> {
public:
  ShootNoteTargeting( ShooterSubsystem* shooter, IntakeSubsystem* intake, 
                      ArmSubsystem* arm, VisionSubsystem* vision );

  void Initialize() override;

  void Execute() override;

  void End(bool interrupted) override;

  bool IsFinished() override;

private:
    ShooterSubsystem* m_shooter;
    IntakeSubsystem* m_intake;
    ArmSubsystem* m_arm;
    VisionSubsystem* m_vision;

    bool readyToShoot;
    bool noTargets;

    units::second_t spin_start;
};

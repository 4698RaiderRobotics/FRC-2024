// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc2/command/CommandHelper.h>
#include <frc2/command/SequentialCommandGroup.h>

#include "subsystems/SwerveDriveSubsystem.h"
#include "subsystems/IntakeSubsystem.h"
#include "subsystems/ShooterSubsystem.h"
#include "subsystems/ArmSubsystem.h"
#include "subsystems/VisionSubsystem.h"

class OnePieceAuto
    : public frc2::CommandHelper<frc2::SequentialCommandGroup,
                                 OnePieceAuto> {
 public:
  OnePieceAuto(SwerveDriveSubsystem* swerve, ShooterSubsystem* shooter, IntakeSubsystem* intake, 
                      ArmSubsystem* arm, VisionSubsystem* vision);
};

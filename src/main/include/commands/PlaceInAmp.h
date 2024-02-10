// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc2/command/CommandHelper.h>
#include <frc2/command/SequentialCommandGroup.h>

#include "subsystems/ArmSubsystem.h"
#include "subsystems/ElevatorSubsystem.h"
#include "subsystems/IntakeSubsystem.h"
#include "subsystems/SwerveDriveSubsystem.h"

class PlaceInAmp
    : public frc2::CommandHelper<frc2::SequentialCommandGroup,
                                 PlaceInAmp> {
 public:
  PlaceInAmp(SwerveDriveSubsystem* swerveDrive, ElevatorSubsystem* elevator, IntakeSubsystem* intake, ArmSubsystem* arm);
};

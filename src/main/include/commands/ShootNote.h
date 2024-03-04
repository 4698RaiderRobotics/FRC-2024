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

class ShootNote
    : public frc2::CommandHelper<frc2::SequentialCommandGroup,
                                 ShootNote> {
 public:
  ShootNote(SwerveDriveSubsystem* swerve, ShooterSubsystem* shooter, IntakeSubsystem* intake, ArmSubsystem* arm, 
            units::degree_t shooterAngle, units::degree_t armAngle, units::degree_t wristAngle);
};

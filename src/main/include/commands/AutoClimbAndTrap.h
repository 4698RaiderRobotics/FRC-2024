// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc2/command/CommandHelper.h>
#include <frc2/command/SequentialCommandGroup.h>

class SwerveDriveSubsystem;
class IntakeSubsystem;
class ArmSubsystem;
class ElevatorSubsystem;
class ClimberSubsystem;
class ShooterSubsystem;
class VisionSubsystem;

class AutoClimbAndTrap
    : public frc2::CommandHelper<frc2::SequentialCommandGroup, AutoClimbAndTrap> {
 public:
  AutoClimbAndTrap(SwerveDriveSubsystem* drive, IntakeSubsystem* intake, ArmSubsystem* arm, ElevatorSubsystem *elevator,
                   ClimberSubsystem *climb, ShooterSubsystem* shooter, VisionSubsystem *vision);

};

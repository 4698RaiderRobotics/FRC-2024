// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc2/command/CommandHelper.h>
#include <frc2/command/SequentialCommandGroup.h>

class IntakeSubsystem;
class ShooterSubsystem;
class ArmSubsystem;
class ClimberSubsystem;
class ElevatorSubsystem;

class ClimbAndTrap
    : public frc2::CommandHelper<frc2::SequentialCommandGroup,
                                 ClimbAndTrap> {
 public:
  ClimbAndTrap(ShooterSubsystem* shooter, IntakeSubsystem* intake, ClimberSubsystem *climber, 
                      ArmSubsystem* arm, ElevatorSubsystem *elevator);
};

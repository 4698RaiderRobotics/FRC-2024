// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "commands/ClimbAndTrap.h"

#include <frc2/command/InstantCommand.h>
#include <frc2/command/WaitCommand.h>
#include <frc2/command/ParallelCommandGroup.h>

#include "commands/ChangeArmAngle.h"
#include "commands/ChangeWristAngle.h"
#include "commands/ChangeElevatorHeight.h"
#include "commands/ChangeShooterAngle.h"
#include "commands/Climb.h"

// NOTE:  Consider using this command inline, rather than writing a subclass.
// For more information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
ClimbAndTrap::ClimbAndTrap(ShooterSubsystem* shooter, IntakeSubsystem* intake, ClimberSubsystem *climber, 
                      ArmSubsystem* arm, ElevatorSubsystem *elevator) {
  // Add your commands here, e.g.
  // AddCommands(FooCommand{}, BarCommand{});
  SetName( "ClimbAndTrap" );

  AddCommands(
    Climb(climber),
    ChangeShooterAngle(shooter, 60_deg),
    frc2::SequentialCommandGroup(ChangeArmAngle(arm, 69_deg), ChangeWristAngle(arm, 63_deg)),
    frc2::WaitCommand(0.5_s),
    frc2::InstantCommand([this, intake] {intake->SpinIntake(-0.5);}, {intake}),
    frc2::WaitCommand(5_s),
    frc2::InstantCommand([this, intake] {intake->SpinIntake(0.0);}, {intake})
  );
}

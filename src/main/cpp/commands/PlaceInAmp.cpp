// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "commands/PlaceInAmp.h"

#include "commands/ChangeArmAngle.h"
#include "commands/ChangeWristAngle.h"
#include "commands/ChangeElevatorHeight.h"

#include "Constants.h"

#include <frc2/command/InstantCommand.h>
#include <frc2/command/WaitCommand.h>


// NOTE:  Consider using this command inline, rather than writing a subclass.
// For more information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
PlaceInAmp::PlaceInAmp(SwerveDriveSubsystem* swerveDrive, ElevatorSubsystem* elevator, IntakeSubsystem* intake, ArmSubsystem* arm) {
  // Add your commands here, e.g.
  // AddCommands(FooCommand{}, BarCommand{});

  SetName( "PlaceInAmp" );

  AddCommands(
    // Drive to amp
    frc2::SequentialCommandGroup(ChangeArmAngle(arm, 75_deg), ChangeWristAngle(arm, 90_deg)),
    ChangeElevatorHeight(elevator, 22_in),
    frc2::SequentialCommandGroup(ChangeArmAngle(arm, 75_deg), ChangeWristAngle(arm, 117_deg)),
    frc2::InstantCommand([this, intake] {intake->SpinIntake(0.5);}, {intake}),
    frc2::WaitCommand(0.2_s),
    frc2::InstantCommand([this, intake] {intake->SpinIntake(0.0);}, {intake}),
    frc2::SequentialCommandGroup(ChangeArmAngle(arm, 75_deg), ChangeWristAngle(arm, 90_deg)),
    ChangeElevatorHeight(elevator, 0_m),
    frc2::SequentialCommandGroup(ChangeArmAngle(arm, 170_deg), ChangeWristAngle(arm, 35_deg))
  );
}

// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "commands/PickUpNote.h"

#include "Constants.h"

#include "commands/ChangeArmAngle.h"
#include "commands/ChangeWristAngle.h"
#include "commands/IntakeNote.h"
#include "commands/ChangeElevatorHeight.h"

#include <frc2/command/SequentialCommandGroup.h>

// NOTE:  Consider using this command inline, rather than writing a subclass.
// For more information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
PickUpNote::PickUpNote(SwerveDriveSubsystem* drive, IntakeSubsystem* intake, ArmSubsystem* arm, ElevatorSubsystem *elevator) {
  // Add your commands here, e.g.
  // AddCommands(FooCommand{}, BarCommand{});

  SetName( "PickUpNote" );

  AddCommands(
    // Limelight data
    // Drive to note
    ChangeElevatorHeight(elevator, 0_in),
    frc2::SequentialCommandGroup(ChangeArmAngle(arm, physical::kArmGroundPickUpAngle), ChangeWristAngle(arm, physical::kWristGroundPickUpAngle)),
    IntakeNote(intake),
    frc2::SequentialCommandGroup(ChangeArmAngle(arm, physical::kArmPassiveAngle), ChangeWristAngle(arm, physical::kWristPassiveAngle))
  );
}

// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "commands/PickUpNote.h"

#include "commands/ChangeArmAngle.h"
#include "commands/IntakeNote.h"

// NOTE:  Consider using this command inline, rather than writing a subclass.
// For more information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
PickUpNote::PickUpNote(SwerveDriveSubsystem* drive, IntakeSubsystem* intake, ArmSubsystem* arm) {
  // Add your commands here, e.g.
  // AddCommands(FooCommand{}, BarCommand{});
  AddCommands(
    // Limelight data
    // Drive to note
    ChangeArmAngle(arm, 45_deg),
    IntakeNote(intake),
    ChangeArmAngle(arm, 90_deg)
  );
}

// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "commands/autonomous/OnePieceTaxiAuto.h"

#include "commands/ShootNoteTargeting.h"
#include "commands/ProfiledDriveToPose.h"

// NOTE:  Consider using this command inline, rather than writing a subclass.
// For more information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
OnePieceTaxiAuto::OnePieceTaxiAuto(SwerveDriveSubsystem* swerve, ShooterSubsystem* shooter, IntakeSubsystem* intake, 
                      ArmSubsystem* arm, ElevatorSubsystem* elevator, VisionSubsystem* vision) {
  // Add your commands here, e.g.
  // AddCommands(FooCommand{}, BarCommand{});
  AddCommands(
    ShootNoteTargeting(swerve, shooter, intake, arm, vision),
    ProfiledDriveToPose(swerve, {0_m, -0.5_m, 0_deg})
  );
}

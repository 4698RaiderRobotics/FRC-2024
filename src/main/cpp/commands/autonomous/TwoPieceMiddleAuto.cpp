// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "commands/autonomous/TwoPieceMiddleAuto.h"

#include <frc2/command/ParallelCommandGroup.h>

#include "commands/ShootNoteTargeting.h"
#include "commands/ProfiledDriveToPose.h"
#include "commands/PickUpNote.h"
#include "commands/autonomous/FollowTrajectory.h"

// NOTE:  Consider using this command inline, rather than writing a subclass.
// For more information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
TwoPieceMiddleAuto::TwoPieceMiddleAuto(SwerveDriveSubsystem* swerve, ShooterSubsystem* shooter, IntakeSubsystem* intake, 
                      ArmSubsystem* arm, ElevatorSubsystem* elevator, VisionSubsystem* vision)
{
  // Add your commands here, e.g.
  // AddCommands(FooCommand{}, BarCommand{});
  AddCommands(
    ShootNoteTargeting(swerve, shooter, intake, arm, vision),
    frc2::ParallelCommandGroup(FollowTrajectory(swerve, swerve->exampleTraj, 0_deg), PickUpNote(swerve, intake, arm, elevator)),
    ShootNoteTargeting(swerve, shooter, intake, arm, vision)
  );
}

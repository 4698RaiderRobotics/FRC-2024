// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "commands/autonomous/TwoPieceSideAuto.h"

#include <frc2/command/ParallelCommandGroup.h>

#include "commands/ShootNoteTargeting.h"
#include "commands/ProfiledDriveToPose.h"
#include "commands/PickUpNote.h"

// NOTE:  Consider using this command inline, rather than writing a subclass.
// For more information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
TwoPieceSideAuto::TwoPieceSideAuto(SwerveDriveSubsystem* swerve, ShooterSubsystem* shooter, IntakeSubsystem* intake, 
                      ArmSubsystem* arm, ElevatorSubsystem* elevator, VisionSubsystem* vision, bool isStartingLeft) {
  // Add your commands here, e.g.
  // AddCommands(FooCommand{}, BarCommand{});
  frc::Pose2d targetPose;
  if(isStartingLeft) {
    targetPose = {1_m, 0.5_m, 30_deg};
  } else {
    targetPose = {1_m, -0.5_m, -30_deg};
  }

  AddCommands(
    ShootNoteTargeting(swerve, shooter, intake, arm, vision),
    frc2::ParallelCommandGroup(ProfiledDriveToPose(swerve, targetPose), PickUpNote(swerve, intake, arm, elevator)),
    ShootNoteTargeting(swerve, shooter, intake, arm, vision)
  );
}

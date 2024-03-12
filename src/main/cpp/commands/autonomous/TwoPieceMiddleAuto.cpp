// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "commands/autonomous/TwoPieceMiddleAuto.h"

#include <frc2/command/ParallelCommandGroup.h>
#include <frc2/command/WaitCommand.h>
#include <frc/DriverStation.h>

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
  units::meter_t xCoord;
  units::degree_t theta;
  if(frc::DriverStation::GetAlliance() == frc::DriverStation::Alliance::kBlue) {
    xCoord = 3_m;
    theta = 0_deg;
  } else {
    xCoord = 14_m;
    theta = 180_deg;
  }
  AddCommands(
    ShootNoteTargeting(swerve, shooter, intake, arm, elevator, vision),
    // frc2::ParallelCommandGroup(frc2::SequentialCommandGroup(frc2::WaitCommand(0.5_s), ProfiledDriveToPose(swerve, {xCoord, 5.5_m, theta})), 
    //                            PickUpNote(swerve, intake, arm, elevator)),
    frc2::ParallelCommandGroup(frc2::SequentialCommandGroup(frc2::WaitCommand(0.5_s), ProfiledDriveToPose(swerve, {1.2_m, 0_m, 0_deg})), 
                               PickUpNote(swerve, intake, arm, elevator)),
    ProfiledDriveToPose(swerve, {-1.2_m, 0_m, 0_deg}),
    ShootNoteTargeting(swerve, shooter, intake, arm, elevator, vision)
  );
}

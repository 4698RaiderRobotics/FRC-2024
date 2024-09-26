// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include <frc/DriverStation.h>
#include <frc2/command/WaitCommand.h>
#include <frc2/command/InstantCommand.h>
#include <frc2/command/ParallelCommandGroup.h>

#include "subsystems/ArmSubsystem.h"
#include "subsystems/ElevatorSubsystem.h"
#include "subsystems/IntakeSubsystem.h"
#include "subsystems/SwerveDriveSubsystem.h"
#include "subsystems/VisionSubsystem.h"

#include "commands/MoveToAndPlaceInAmp.h"
#include "commands/ProfiledDriveToPose.h"
#include "commands/ChangeArmAngle.h"
#include "commands/ChangeWristAngle.h"
#include "commands/ChangeElevatorHeight.h"
#include "commands/GotoRestPosition.h"

#include "Constants.h"


// NOTE:  Consider using this command inline, rather than writing a subclass.
// For more information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
MoveToAndPlaceInAmp::MoveToAndPlaceInAmp(SwerveDriveSubsystem* drive, IntakeSubsystem* intake, ArmSubsystem* arm, ElevatorSubsystem *elevator,
                                        VisionSubsystem* vision) {
  // Add your commands here, e.g.
  // AddCommands(FooCommand{}, BarCommand{});

  SetName( "MoveToAndPlaceInAmp" );

  frc::Pose2d targetPose;

  if(frc::DriverStation::GetAlliance() == frc::DriverStation::Alliance::kBlue) {
    targetPose = {72.5_in, 323_in - 20_in, 90_deg};
  } else {
    targetPose = {578.77_in, 323_in - 20_in, 90_deg};
  }

  AddCommands(
    ProfiledDriveToPose(drive, vision, targetPose),
    frc2::SequentialCommandGroup(
        ChangeArmAngle(arm, physical::kArmAmpAngle), 
        ChangeWristAngle(arm, physical::kWristAmpAngle)),
    ChangeElevatorHeight(elevator, 19_in),
    frc2::InstantCommand([this, drive] {drive->Drive({0_mps, 1_mps, 0_deg_per_s});}, {drive}),
    frc2::WaitCommand(0.5_s),
    frc2::InstantCommand([this, drive] {drive->Drive({0_mps, 0_mps, 0_deg_per_s});}, {drive}),
    // ProfiledDriveToPose(drive, vision, {targetPose.X(), targetPose.Y() + 6_in, targetPose.Rotation()}),
    frc2::SequentialCommandGroup(ChangeArmAngle(arm, physical::kArmAmpAngle), ChangeWristAngle(arm, physical::kWristAmpSpitAngle)),
    frc2::InstantCommand([this, intake] {intake->SpinIntake(0.75);}, {intake}),
    frc2::WaitCommand(0.5_s),
    frc2::InstantCommand([this, intake] {intake->SpinIntake(0.0);}, {intake}),
    frc2::ParallelCommandGroup(
      frc2::SequentialCommandGroup(ChangeArmAngle(arm, physical::kArmAmpDropAngle), ChangeWristAngle(arm, physical::kWristAmpDropAngle)),
      ProfiledDriveToPose(drive, vision, {targetPose.X(), targetPose.Y() - 3_in, targetPose.Rotation()})),
    ChangeElevatorHeight(elevator, 0_m),
    GoToRestPosition( arm, elevator, intake )
  );
}

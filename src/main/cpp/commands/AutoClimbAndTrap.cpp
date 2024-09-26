// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "commands/AutoClimbAndTrap.h"
#include <frc/DriverStation.h>

#include "commands/ProfiledDriveToPose.h"
#include "commands/Climb.h"
#include "commands/ChangeArmAngle.h"
#include "commands/ChangeWristAngle.h"
#include "commands/ChangeElevatorHeight.h"
#include "commands/ChangeShooterAngle.h"

#include <frc2/command/WaitCommand.h>
#include <frc2/command/InstantCommand.h>
#include <frc2/command/ParallelCommandGroup.h>

// NOTE:  Consider using this command inline, rather than writing a subclass.
// For more information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
AutoClimbAndTrap::AutoClimbAndTrap(SwerveDriveSubsystem* drive, IntakeSubsystem* intake, ArmSubsystem* arm, ElevatorSubsystem *elevator,
                                   ClimberSubsystem *climb, ShooterSubsystem *shooter, VisionSubsystem *vision ) {
  // Add your commands here, e.g.
  // AddCommands(FooCommand{}, BarCommand{});

  SetName( "AutoClimbAndTrap" );

  frc::Pose2d targetPose;
  frc::Pose2d hook_pose;
  frc::Pose2d tag_pose;

  frc::Pose2d redClimbTags[3] = {
    {468.69_in, 177.1_in, 60_deg},
    {468.69_in, 146.19_in, 300_deg},
    {441.74_in, 161.62_in, 180_deg}
  };

  frc::Pose2d blueClimbTags[3] = {
    {209.48_in, 161.62_in, 0_deg},
    {182.73_in, 177.10_in, 120_deg},
    {182.73_in, 146.19_in, 240_deg}
  };

  // if(frc::DriverStation::GetAlliance() == frc::DriverStation::Alliance::kBlue) {
  //   targetPose = {72.5_in, 323_in - 20_in, 90_deg};
  // } else {
  //   targetPose = {578.77_in, 323_in - 20_in, 90_deg};
  // }


  if(frc::DriverStation::GetAlliance() == frc::DriverStation::Alliance::kRed) {
    tag_pose = drive->GetPose().Nearest(std::span<frc::Pose2d>(redClimbTags, 3));
    fmt::print("Nearest To: {}, {}, {}\n", tag_pose.X(), tag_pose.Y(), tag_pose.Rotation().Degrees());
  } else if(frc::DriverStation::GetAlliance() == frc::DriverStation::Alliance::kBlue) {
    tag_pose = drive->GetPose().Nearest(std::span<frc::Pose2d>(blueClimbTags, 3));
    fmt::print("Nearest To: {}, {}, {}\n", tag_pose.X(), tag_pose.Y(), tag_pose.Rotation().Degrees());
  }
  units::inch_t starting_offset = 20_in;
  units::inch_t hook_offset = 8_in;
  targetPose = {tag_pose.X() + starting_offset * units::math::cos( tag_pose.Rotation().Degrees() ),
                tag_pose.Y() + starting_offset * units::math::sin( tag_pose.Rotation().Degrees() ),
                tag_pose.Rotation().Degrees() - 180_deg};
  hook_pose = {tag_pose.X() + hook_offset * units::math::cos( tag_pose.Rotation().Degrees() ),
                tag_pose.Y() + hook_offset * units::math::sin( tag_pose.Rotation().Degrees() ),
                tag_pose.Rotation().Degrees() - 180_deg};
  

  AddCommands(
      // Drive to the initial target pose and raise the climber hooks
    frc2::ParallelCommandGroup( 
      ProfiledDriveToPose(drive, vision, targetPose),
      Climb( climb, physical::kClimberMaxHeight ),
      ChangeShooterAngle(shooter, 60_deg)
    ),

    // Put the arm up and in
    frc2::SequentialCommandGroup(
      ChangeArmAngle(arm, 110_deg), 
      ChangeWristAngle(arm, 95_deg)
    ),

      // Drive forward and then drop the climber hooks.
    ProfiledDriveToPose(drive, vision, hook_pose),
    frc2::WaitCommand(1_s),
    Climb( climb, physical::kClimberMidHeight ),

      // Put the arm up
    frc2::SequentialCommandGroup(
      ChangeArmAngle(arm, 70_deg), 
      ChangeWristAngle(arm, 90_deg)
    ),
    ChangeElevatorHeight(elevator, physical::kElevatorTrapHeight),

      // Climb the rest of the way and deposit the note.
    Climb( climb ),
    frc2::SequentialCommandGroup(ChangeArmAngle(arm, 70_deg), ChangeWristAngle(arm, 63_deg)),
    frc2::WaitCommand(2_s),
    frc2::InstantCommand([this, intake] {intake->SpinIntake(-0.5);}, {intake}),
    frc2::WaitCommand(5_s),
    frc2::InstantCommand([this, intake] {intake->SpinIntake(0.0);}, {intake})
  );
}

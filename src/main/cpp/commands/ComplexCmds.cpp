

#include <units/length.h>

#include <frc/DriverStation.h>
#include <frc/geometry/Pose2d.h>
#include <frc2/command/Commands.h>

#include "Constants.h"

#include "commands/ComplexCmds.h"

#include "subsystems/ArmSubsystem.h"
#include "subsystems/ElevatorSubsystem.h"
#include "subsystems/IntakeSubsystem.h"
#include "subsystems/ClimberSubsystem.h"
#include "subsystems/SwerveDriveSubsystem.h"
#include "subsystems/ShooterSubsystem.h"
#include "subsystems/VisionSubsystem.h"

#include "commands/IntakeNote.h"
#include "commands/GoToRestPosition.h"
#include "commands/ProfiledDriveToPose.h"


frc2::CommandPtr PickUpNote( IntakeSubsystem* intake, ArmSubsystem* arm, ElevatorSubsystem *elevator) {
  return frc2::cmd::Sequence(
    elevator->ChangeHeight( 0_in ),
    arm->MoveJoints( physical::kArmGroundPickUpAngle, physical::kWristGroundPickUpAngle ),
    IntakeNote(intake).ToPtr(),
    GoToRestPosition(arm, elevator, intake).ToPtr()
  ).WithName( "PickupNote" );
}


frc2::CommandPtr DriverPlaceInAmp( ArmSubsystem* arm, ElevatorSubsystem *elevator, IntakeSubsystem* intake ) {
  return frc2::cmd::Sequence(
    arm->MoveJoints( physical::kArmAmpAngle, physical::kWristAmpSpitAngle),
    frc2::cmd::RunOnce([intake] {intake->SpinIntake(0.5);}, {intake}),
    frc2::cmd::Wait(0.5_s),
    frc2::cmd::RunOnce([intake] {intake->SpinIntake(0.0);}, {intake}),
    arm->MoveJoints( physical::kArmAmpDropAngle, physical::kWristAmpDropAngle),
    elevator->ChangeHeight( 0_m ),
    GoToRestPosition( arm, elevator, intake ).ToPtr()
  ).WithName( "DriverPlaceInAmp");
}


frc2::CommandPtr MoveToAndPlaceInAmpImpl(SwerveDriveSubsystem* drive, IntakeSubsystem* intake, ArmSubsystem* arm, ElevatorSubsystem *elevator,
                                        VisionSubsystem* vision) {
  frc::Pose2d targetPose;

  if(frc::DriverStation::GetAlliance() == frc::DriverStation::Alliance::kBlue) {
    targetPose = {72.5_in, 323_in - 20_in, 90_deg};
  } else {
    targetPose = {578.77_in, 323_in - 20_in, 90_deg};
  }

  return frc2::cmd::Sequence( 
    ProfiledDriveToPose(drive, vision, targetPose).ToPtr(),
    arm->MoveJoints( physical::kArmAmpAngle, physical::kWristAmpAngle),
    elevator->ChangeHeight( 19_in ),
    frc2::cmd::RunOnce([drive] {drive->Drive({0_mps, 1_mps, 0_deg_per_s});}, {drive}),
    frc2::cmd::Wait(0.5_s),
    frc2::cmd::RunOnce([drive] {drive->Drive({0_mps, 0_mps, 0_deg_per_s});}, {drive}),
    arm->MoveJoints( physical::kArmAmpAngle, physical::kWristAmpSpitAngle),
    frc2::cmd::RunOnce([intake] {intake->SpinIntake(0.75);}, {intake}),
    frc2::cmd::Wait(0.5_s),
    frc2::cmd::RunOnce([intake] {intake->SpinIntake(0.0);}, {intake}),
    frc2::cmd::Parallel(
      arm->MoveJoints( physical::kArmAmpDropAngle, physical::kWristAmpDropAngle),
      ProfiledDriveToPose( drive, vision, {targetPose.X(), targetPose.Y() - 3_in, targetPose.Rotation()} ).ToPtr()
    ),
    elevator->ChangeHeight( 0_m ),
    GoToRestPosition( arm, elevator, intake ).ToPtr()
  ).WithName( "MoveToAndPlaceInAmp" );
}


frc2::CommandPtr ClimbAndTrap(ShooterSubsystem* shooter, IntakeSubsystem* intake, ClimberSubsystem *climber, 
                      ArmSubsystem* arm, ElevatorSubsystem *elevator) {
  return frc2::cmd::Sequence(
    climber->MoveHooks( 0_in ),
    shooter->ChangeAngle( 60_deg ),
    arm->MoveJoints( 69_deg, 63_deg ),
    frc2::cmd::Wait(0.5_s),
    frc2::cmd::RunOnce([intake] {intake->SpinIntake(-0.5);}, {intake}),
    frc2::cmd::Wait(5_s),
    frc2::cmd::RunOnce([intake] {intake->SpinIntake(0.0);}, {intake})
  ).WithName( "ClimbAndTrap" );
}


frc2::CommandPtr AutoClimbAndTrapImpl( SwerveDriveSubsystem* drive, IntakeSubsystem* intake, ArmSubsystem* arm, ElevatorSubsystem *elevator,
                                   ClimberSubsystem *climb, ShooterSubsystem* shooter, VisionSubsystem *vision ) {

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

  if(frc::DriverStation::GetAlliance() == frc::DriverStation::Alliance::kRed) {
    tag_pose = drive->GetPose().Nearest(std::span<frc::Pose2d>(redClimbTags, 3));
    fmt::print("Nearest To: {}, {}, {}\n", tag_pose.X(), tag_pose.Y(), tag_pose.Rotation().Degrees());
  } else {
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
  

  return frc2::cmd::Sequence(
      // Drive to the initial target pose and raise the climber hooks
    frc2::cmd::Parallel( 
      ProfiledDriveToPose(drive, vision, targetPose).ToPtr(),
      climb->MoveHooks( physical::kClimberMaxHeight ),
      shooter->ChangeAngle( 60_deg )
    ),

    // Put the arm up and in
    arm->MoveJoints( 110_deg, 95_deg ),

      // Drive forward and then drop the climber hooks.
    ProfiledDriveToPose(drive, vision, hook_pose).ToPtr(),
    frc2::cmd::Wait(1_s),
    climb->MoveHooks( physical::kClimberMidHeight ),

      // Put the arm up
    arm->MoveJoints( 70_deg, 90_deg ),
    elevator->ChangeHeight( physical::kElevatorTrapHeight ),

      // Climb the rest of the way and deposit the note.
    climb->MoveHooks( 0_in ),

      // Put tip the intake toward the trap
    arm->MoveJoints( 70_deg, 63_deg ),

    frc2::cmd::Wait(2_s),
    frc2::cmd::RunOnce([intake] {intake->SpinIntake(-0.5);}, {intake}),
    frc2::cmd::Wait(5_s),
    frc2::cmd::RunOnce([intake] {intake->SpinIntake(0.0);}, {intake})
  ).WithName( "AutoClimbAndTrap" );
}


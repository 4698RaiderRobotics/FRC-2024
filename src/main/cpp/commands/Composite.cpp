

#include <units/length.h>

#include <frc/DriverStation.h>
#include <frc/geometry/Pose2d.h>
#include <frc2/command/Commands.h>

#include "Constants.h"

#include "commands/Composite.h"

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
    arm->MoveJoints( physical::kArmRaiseAngle, physical::kWristAmpSpitAngle),
    frc2::cmd::RunOnce([intake] {intake->SpinIntake(0.5);}, {intake}),
    frc2::cmd::Wait(0.5_s),
    frc2::cmd::RunOnce([intake] {intake->SpinIntake(0.0);}, {intake}),
    arm->MoveJoints( physical::kArmAmpDropAngle, physical::kWristAmpDropAngle),
    elevator->ChangeHeight( 0_m ),
    GoToRestPosition( arm, elevator, intake ).ToPtr()
  ).WithName( "DriverPlaceInAmp");
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


MoveToAndPlaceInAmp::MoveToAndPlaceInAmp( SwerveDriveSubsystem* d, IntakeSubsystem* i, ArmSubsystem* a,
                         ElevatorSubsystem *e, VisionSubsystem* v) 
                         : drive{d}, intake{i}, arm{a}, elevator{e}, vision{v} {

  amp_tags[0] = {578.77_in, 323_in - 20_in, 90_deg}; // Blue Amp, Tag #5
  amp_tags[1] = {72.5_in, 323_in - 20_in, 90_deg};   // Blue Amp, Tag #6

  for( int i=0; i<2; ++i ) {
    frc::Pose2d targetPose = amp_tags[i];
    
    commands.push_back( std::move( frc2::cmd::Sequence( 
      ProfiledDriveToPose(drive, vision, targetPose).ToPtr(),
      arm->MoveJoints( physical::kArmRaiseAngle, physical::kWristRaiseAngle),
      elevator->ChangeHeight( physical::kElevatorAmpHeight ),
      frc2::cmd::RunOnce([this] {drive->Drive({0_mps, 1_mps, 0_deg_per_s});}, {drive}),
      frc2::cmd::Wait(0.5_s),
      frc2::cmd::RunOnce([this] {drive->Drive({0_mps, 0_mps, 0_deg_per_s});}, {drive}),
      arm->MoveJoints( physical::kArmRaiseAngle, physical::kWristAmpSpitAngle),
      frc2::cmd::RunOnce([this] {intake->SpinIntake(0.75);}, {intake}),
      frc2::cmd::Wait(0.5_s),
      frc2::cmd::RunOnce([this] {intake->SpinIntake(0.0);}, {intake}),
      frc2::cmd::Parallel(
        arm->MoveJoints( physical::kArmAmpDropAngle, physical::kWristAmpDropAngle),
        frc2::cmd::Sequence(
          frc2::cmd::RunOnce([this] {drive->Drive({0_mps, -1_mps, 0_deg_per_s});}, {drive}),
          frc2::cmd::Wait(0.25_s),
          frc2::cmd::RunOnce([this] {drive->Drive({0_mps, 0_mps, 0_deg_per_s});}, {drive})
        )
        // ProfiledDriveToPose( drive, vision, {targetPose.X(), targetPose.Y() - 3_in, targetPose.Rotation()} ).ToPtr()
      ),
      elevator->ChangeHeight( 0_m ),
      GoToRestPosition( arm, elevator, intake ).ToPtr()
    ).WithName( fmt::format("MoveToAndPlaceInAmp - {} Amp", i==0 ? "Red" : "Blue" ))
    ));
  }
}

void MoveToAndPlaceInAmp::Schedule() {
  frc::Pose2d targetPose;

  if(frc::DriverStation::GetAlliance() == frc::DriverStation::Alliance::kRed) {
      // Red Amp command
    commands[0].Schedule();
  } else {
      // Blue Amp command
    commands[1].Schedule();
  }
}


AutoClimbAndTrap::AutoClimbAndTrap( SwerveDriveSubsystem* d, IntakeSubsystem* i, ArmSubsystem* a,
                  ElevatorSubsystem *e, ClimberSubsystem *c, ShooterSubsystem* s, VisionSubsystem* v) 
    : drive{d}, intake{i}, arm{a}, elevator{e}, climb{c}, shooter{s}, vision{v} {

  stage_tags[0] = {468.69_in, 146.19_in, 300_deg};  // Red tag #11
  stage_tags[1] = {468.69_in, 177.1_in, 60_deg};    // Red tag #12
  stage_tags[2] = {441.74_in, 161.62_in, 180_deg};  // Red tag #13
  stage_tags[3] = {209.48_in, 161.62_in, 0_deg};    // Blue tage #14
  stage_tags[4] = {182.73_in, 177.10_in, 120_deg};  // Blue tage #15
  stage_tags[5] = {182.73_in, 146.19_in, 240_deg};  // Blue tage #16
  
  frc::Pose2d targetPose;
  frc::Pose2d hookPose;
  units::inch_t starting_offset = 20_in;
  units::inch_t hook_offset = 4_in;

  for( int i=0; i<6; ++i ) {
    frc::Pose2d tagPose = stage_tags[i];
    targetPose = {tagPose.X() + starting_offset * units::math::cos( tagPose.Rotation().Degrees() ),
                  tagPose.Y() + starting_offset * units::math::sin( tagPose.Rotation().Degrees() ),
                  tagPose.Rotation().Degrees() - 180_deg};
    hookPose = {tagPose.X() + hook_offset * units::math::cos( tagPose.Rotation().Degrees() ),
                  tagPose.Y() + hook_offset * units::math::sin( tagPose.Rotation().Degrees() ),
                  tagPose.Rotation().Degrees() - 180_deg};

    commands.push_back( std::move( frc2::cmd::Sequence(
          // Drive to the initial target pose and raise the climber hooks
        frc2::cmd::Parallel( 
          ProfiledDriveToPose(drive, vision, targetPose).ToPtr(),
          climb->MoveHooks( physical::kClimberMaxHeight ),
          shooter->ChangeAngle( 60_deg )
        ),

        // Put the arm up and in
        arm->MoveJoints( physical::kArmPreClimbAngle, physical::kWristPreClimbAngle ),

          // Drive forward and then drop the climber hooks.
        ProfiledDriveToPose(drive, vision, hookPose).ToPtr(),
        frc2::cmd::Wait(0.25_s),
        climb->MoveHooks( physical::kClimberMaxHeight - 5.5_in ),
        frc2::cmd::Wait(0.25_s),
        climb->MoveHooks( physical::kClimberMidHeight ),

          // Put the arm up
        arm->MoveJoints( physical::kArmRaiseAngle, physical::kWristTrapSpitAngle),
        elevator->ChangeHeight( physical::kElevatorTrapHeight ),

          // Climb the rest of the way and deposit the note.
        climb->MoveHooks( physical::kClimberMinHeight ),

          // Put tip the intake toward the trap
        // arm->MoveJoints( physical::kArmRaiseAngle, physical::kWristTrapSpitAngle ),

        frc2::cmd::Wait(1_s),
        frc2::cmd::RunOnce([this] {intake->SpinIntake(-0.5);}, {intake}),
        frc2::cmd::Wait(5_s),
        frc2::cmd::RunOnce([this] {intake->SpinIntake(0.0);}, {intake})
      ).WithName( fmt::format( "AutoClimbAndTrap Tag #{}", i+11 ) )
    ) );
  }
}

void AutoClimbAndTrap::Schedule() {

  frc::Pose2d currentPose = drive->GetPose();
  units::inch_t min_distance = 9999999.0_in;
  units::inch_t tag_distance;
  int tag_number_offset;
  int closest_tag = -1;

  if(frc::DriverStation::GetAlliance() == frc::DriverStation::Alliance::kRed) {
    tag_number_offset = 0;
  } else {
    tag_number_offset = 3;
  }

  for( int i=0; i<3; ++i ) {
    tag_distance = currentPose.RelativeTo(stage_tags[ i + tag_number_offset ]).Translation().Norm();
    if( tag_distance < min_distance ) {
      min_distance = tag_distance;
      closest_tag = i + tag_number_offset;
    }
  }

  fmt::print("Nearest To Tag #{}, dist = {}\n", closest_tag + 11, min_distance );

  if( closest_tag >=0 && closest_tag < 6 ) {
    commands[closest_tag].Schedule();
  } else {
    fmt::print( "  CANNOT FIND COMMAND FOR TAG #{}\n", closest_tag + 11 );
  }
}

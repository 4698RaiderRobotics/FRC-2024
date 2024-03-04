// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "commands/ShootNoteTargeting.h"

#include "Constants.h"

#include "commands/ChangeShooterAngle.h"
#include "commands/SpinShooter.h"
#include "commands/ChangeArmAngle.h"
#include "commands/ChangeWristAngle.h"

#include <frc2/command/InstantCommand.h>
#include <frc2/command/WaitCommand.h>
#include <frc2/command/ParallelCommandGroup.h>


ShootNoteTargeting::ShootNoteTargeting( ShooterSubsystem* shooter, 
                      IntakeSubsystem* intake, ArmSubsystem* arm, 
                      VisionSubsystem* vision ) :
                      m_shooter{shooter}, m_intake{intake}, m_arm{arm}, m_vision{vision}
{
  AddRequirements({shooter, intake, arm, vision});
  // Add your commands here, e.g.
  // AddCommands(FooCommand{}, BarCommand{});
  // AddCommands(
  //   // AprilTag data
  //   // Drive to location + spin
  //   frc2::ParallelCommandGroup(SpinShooter(shooter, 2000_rpm),
  //                              frc2::SequentialCommandGroup(ChangeArmAngle(arm, armAngle), ChangeWristAngle(arm, wristAngle))),
  //   ChangeShooterAngle(shooter, shooterAngle), 
  //   frc2::InstantCommand([this, intake] {intake->SpinIntake(-1);}, {intake}),
  //   frc2::WaitCommand(0.5_s),
  //   frc2::InstantCommand([this, intake] {intake->SpinIntake(0.0);}, {intake}),
  //   SpinShooter(shooter, 0_rpm),
  //   ChangeShooterAngle(shooter, 30_deg),
  //   frc2::SequentialCommandGroup(ChangeArmAngle(arm, 170_deg), ChangeWristAngle(arm, 35_deg))
  // );
}

// Called when the command is initially scheduled.
void ShootNoteTargeting::Initialize() {
    // Start the shooter motors and move to the correct arm and wrist positions.
  m_shooter->Spin( 2000_rpm );
  m_arm->GoToArmAngle( m_shooter->GetShooter_ArmAngle() );
  m_arm->GoToWristAngle( m_shooter->GetShooter_WristAngle() );

  readyToShoot = false;
  noTargets = false;
}

// Called repeatedly when this Command is scheduled to run
void ShootNoteTargeting::Execute() {
  double t_yaw, t_pitch;

  if( !readyToShoot ) {
      // Set the shooter angle based on the position of the apriltag
    if( m_vision->GetAprilTagPosition( 6, t_yaw, t_pitch ) ) {
      units::degree_t targetAngle = 45_deg;
      fmt::print( "Tracking target, pitch = {}, moving shooter to {}\n", t_pitch, targetAngle );

      m_shooter->GoToAngle( targetAngle );
      m_arm->GoToArmAngle( m_shooter->GetShooter_ArmAngle() );
      m_arm->GoToWristAngle( m_shooter->GetShooter_WristAngle() );

      if( m_shooter->IsAtGoal() && m_arm->IsAtGoal() ) {
        readyToShoot = true;
        m_intake->SpinIntake( -1 );
        spin_start = frc::Timer::GetFPGATimestamp();
      }
    } else {
      noTargets = true;
    }
  } else {
      // We are currently shooting
    if( frc::Timer::GetFPGATimestamp() - spin_start > 0.5_s ) {
        // We are done shooting...
      m_intake->SpinIntake( 0.0 );
      noTargets = true;  // No ammo...
    }
  }
}

// Called once the command ends or is interrupted.
void ShootNoteTargeting::End(bool interrupted) {
  m_shooter->Spin( 0_rpm );
  m_shooter->GoToAngle( 30_deg );
  m_arm->GoToArmAngle( 170_deg );
  m_arm->GoToWristAngle( 35_deg );
}

// Returns true when the command should end.
bool ShootNoteTargeting::IsFinished() {

  return noTargets;
}

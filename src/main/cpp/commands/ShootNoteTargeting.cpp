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

#include <frc/DriverStation.h>


ShootNoteTargeting::ShootNoteTargeting( SwerveDriveSubsystem* swerve, ShooterSubsystem* shooter, 
                      IntakeSubsystem* intake, ArmSubsystem* arm, 
                      VisionSubsystem* vision) :
                      m_drive{swerve}, m_shooter{shooter}, m_intake{intake}, m_arm{arm}, m_vision{vision}
{
  allowDriving = false;
  m_x_axis = nullptr;
  m_y_axis = nullptr;

  AddRequirements({m_drive, m_shooter, m_intake, m_arm, m_vision});
}

ShootNoteTargeting::ShootNoteTargeting( SwerveDriveSubsystem* swerve, ShooterSubsystem* shooter, 
                      IntakeSubsystem* intake, ArmSubsystem* arm, 
                      VisionSubsystem* vision, ControllerAxis *x_axis, ControllerAxis *y_axis ) :
                      m_drive{swerve}, m_shooter{shooter}, m_intake{intake}, m_arm{arm}, m_vision{vision}
{
  allowDriving = true;
  m_x_axis = x_axis;
  m_y_axis = y_axis;
  AddRequirements({m_drive, m_shooter, m_intake, m_arm, m_vision});
}

// Called when the command is initially scheduled.
void ShootNoteTargeting::Initialize() {
    // Start the shooter motors and move to the correct arm and wrist positions.
  m_shooter->Spin( 2000_rpm );
  // m_arm->GoToArmAngle( m_shooter->GetShooter_ArmAngle() );
  m_arm->GoToWristAngle( m_shooter->GetShooter_WristAngle() );

  fmt::print("Setting Arm / Wrist to {}/{}\n",  m_shooter->GetShooter_ArmAngle(), m_shooter->GetShooter_WristAngle() );

  if(frc::DriverStation::GetAlliance().value() == frc::DriverStation::Alliance::kRed) {
    targetingID = 4;
  } else {
    targetingID = 7;
  }

  readyToShoot = false;
  noTargets = false;
}

// Called repeatedly when this Command is scheduled to run
void ShootNoteTargeting::Execute() {
  double t_yaw, t_pitch;

  if( !readyToShoot ) {
      // Set the shooter angle based on the position of the apriltag
    if( m_vision->GetAprilTagPosition( targetingID, t_yaw, t_pitch ) ) {
      units::degree_t targetAngle = pitchToAngle.lookup(t_pitch) * 1_deg;
      // fmt::print( "Tracking target, pitch = {}, moving shooter to {}\n", t_pitch, targetAngle );

      m_shooter->GoToAngle( targetAngle );
      if( m_arm->GetWristAngle() > 80_deg ) {
         m_arm->GoToArmAngle( m_shooter->GetShooter_ArmAngle() );
      }
      m_arm->GoToWristAngle( m_shooter->GetShooter_WristAngle() );

      if( allowDriving ) {
        m_drive->ArcadeDrive( m_x_axis->GetAxis(), m_y_axis->GetAxis(), -t_yaw * 0.02 );
      } else {
        m_drive->ArcadeDrive( 0, 0, -t_yaw * 0.02 );
      }

      fmt::print( "    AtGoal:  shooter({}), arm({}), yaw({})\n", m_shooter->IsAtGoal(), m_arm->IsAtGoal( 6_deg ), fabs(t_yaw) < 1.5 );
      if( m_shooter->IsAtGoal() && m_arm->IsAtGoal(6_deg) && fabs(t_yaw) < 1.5 ) {
        fmt::print( "ShootNoteTargeting(), pitch({}), Shooter({}), Arm({}), Wrist({})\n", t_pitch, targetAngle,
                     m_shooter->GetShooter_ArmAngle(), m_shooter->GetShooter_WristAngle()  );
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

  fmt::print( "ShootNoteTargeting::End interrupted({}), noTargets({})\n", interrupted, noTargets );
  m_shooter->Spin( 0_rpm );
  m_shooter->GoToAngle( 30_deg );
  m_arm->GoToArmAngle( 170_deg );
  m_arm->GoToWristAngle( 35_deg );
}

// Returns true when the command should end.
bool ShootNoteTargeting::IsFinished() {

  return noTargets;
}

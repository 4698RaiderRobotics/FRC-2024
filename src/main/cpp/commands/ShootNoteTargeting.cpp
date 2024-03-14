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

#include "DataLogger.h"

ShootNoteTargeting::ShootNoteTargeting( SwerveDriveSubsystem* swerve, ShooterSubsystem* shooter, 
                      IntakeSubsystem* intake, ArmSubsystem* arm, ElevatorSubsystem* elev, 
                      VisionSubsystem* vision, ControllerAxis *x_axis, ControllerAxis *y_axis ) :
                      m_drive{swerve}, m_shooter{shooter}, m_intake{intake}, m_arm{arm}, m_elev{elev},
                      m_vision{vision}, m_x_axis{x_axis}, m_y_axis{y_axis}
{
  SetName( "ShootNoteTargeting" );

  if( x_axis != nullptr ) {
    allowDriving = true;
  } else {
    allowDriving = false;
  }

  AddRequirements({m_drive, m_shooter, m_intake, m_arm, m_elev, m_vision});
}

// Called when the command is initially scheduled.
void ShootNoteTargeting::Init() {
    // Start the shooter motors and move to the correct arm and wrist positions.
  m_shooter->Spin( 2000_rpm );
  // m_arm->GoToArmAngle( m_shooter->GetShooter_ArmAngle() );
      m_arm->GoToWristAngle( 180_deg - m_shooter->GetAngle() );

  fmt::print("Setting Arm / Wrist / Elevator to {}/{}/{}\n", 
              m_shooter->GetShooter_ArmAngle(), 180_deg - m_shooter->GetAngle(), 
              m_shooter->GetShooter_ElevatorHeight() );

  if(frc::DriverStation::GetAlliance().value() == frc::DriverStation::Alliance::kRed) {
        // 2D location of AprilTag #4
    targetLocation = {652.73_in, 218.42_in, 180_deg};
  } else {
        // 2D location of AprilTag #7
    targetLocation = {-1.5_in, 218.42_in, 0_deg};
  }

  readyToShoot = false;
  noTargets = false;
}

// Called repeatedly when this Command is scheduled to run
void ShootNoteTargeting::Execute() {
  // double t_yaw, t_pitch;

  units::degree_t arm_tolerance;

  arm_tolerance = 5_deg;

  if( !readyToShoot ) {
      // Set the shooter angle based on the distance between the robot and the speaker.
    frc::Pose2d pose_to_speaker = m_drive->GetPose().RelativeTo( targetLocation );
    units::meter_t dist_to_speaker = pose_to_speaker.Translation().Norm();

    DataLogger::GetInstance().SendNT( "ShootNote/RobotToSpeaker Pose", pose_to_speaker );
    DataLogger::GetInstance().SendNT( "ShootNote/RobotToSpeaker dist", dist_to_speaker.value() );

    if( dist_to_speaker < 4_m  ) {
      units::degree_t geometryAngle = units::math::atan2( 78_in, dist_to_speaker );
      units::degree_t shooterAngle = geometryAngle - 10_deg;

      double t_yaw = pose_to_speaker.Rotation().Degrees().value();

      m_shooter->GoToAngle( shooterAngle );
      if( m_arm->GetWristAngle() > 80_deg ) {
        m_arm->GoToArmAngle( m_shooter->GetShooter_ArmAngle() );
        m_elev->GoToHeight( m_shooter->GetShooter_ElevatorHeight() );
      }
      m_arm->GoToWristAngle( 180_deg - shooterAngle );

      if( allowDriving ) {
        m_drive->ArcadeDrive( m_x_axis->GetAxis(), m_y_axis->GetAxis(), -t_yaw * 0.02 );
      } else {
        m_drive->ArcadeDrive( 0, 0, -t_yaw * 0.02 );
      }

      DataLogger::GetInstance().SendNT( "ShootNote/Shooter Angle Goal", shooterAngle.value() );
      DataLogger::GetInstance().SendNT( "ShootNote/Shooter Angle", m_shooter->GetAngle().value() );
      DataLogger::GetInstance().SendNT( "ShootNote/Arm Angle Goal", m_shooter->GetShooter_ArmAngle().value() );
      DataLogger::GetInstance().SendNT( "ShootNote/Arm Angle", m_arm->GetArmAngle().value() );
      DataLogger::GetInstance().SendNT( "ShootNote/Wrist Angle Goal", 180 - shooterAngle.value() );
      DataLogger::GetInstance().SendNT( "ShootNote/Wrist Angle", m_arm->GetWristAngle().value() );
      DataLogger::GetInstance().SendNT( "ShootNote/Elevator Height", m_elev->GetHeight().value() );
      DataLogger::GetInstance().SendNT( "ShootNote/Arm IsAtGoal", m_arm->IsAtGoal( arm_tolerance ) );
      DataLogger::GetInstance().SendNT( "ShootNote/Shooter IsAtGoal", m_shooter->IsAtGoal() );
      DataLogger::GetInstance().SendNT( "ShootNote/Elevator IsAtGoal", m_elev->IsAtGoal() );
      DataLogger::GetInstance().SendNT( "ShootNote/Target Yaw", t_yaw );

      // fmt::print( "    Arm/Wrist Angles:  atGoal({}), Curr_arm({}), Arm_target({}), Curr_wrist({}), Wrist_target({})\n", 
      //                 m_arm->IsAtGoal( arm_tolerance ), m_arm->GetArmAngle(), m_shooter->GetShooter_ArmAngle(),
      //                  m_arm->GetWristAngle(), 180_deg - targetAngle );
      if( m_shooter->IsAtGoal() && m_arm->IsAtGoal( arm_tolerance ) && fabs(t_yaw) < 1.5 ) {
        // fmt::print( "ShootNoteTargeting(), pitch({}), Shooter({}), Arm({}), Wrist({})\n", t_pitch, targetAngle,
        //              m_shooter->GetShooter_ArmAngle(), 180_deg - targetAngle  );
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
void ShootNoteTargeting::HasEnded(bool interrupted) {
  fmt::print( "ShootNoteTargeting::End interrupted({}), noTargets({})\n", interrupted, noTargets );
  m_shooter->Spin( 0_rpm );
  m_shooter->GoToAngle( 30_deg );
  m_arm->GoToArmAngle( 170_deg );
  m_arm->GoToWristAngle( 35_deg );
  m_elev->GoToHeight(0_in);
}

// Returns true when the command should end.
bool ShootNoteTargeting::IsFinished() {

  return noTargets;
}

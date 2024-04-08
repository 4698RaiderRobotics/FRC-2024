// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include <frc/smartdashboard/SmartDashboard.h>

#include "commands/ProfiledDriveToPose.h"

#include "DataLogger.h"

ProfiledDriveToPose::ProfiledDriveToPose(SwerveDriveSubsystem *swerve, VisionSubsystem *vision, frc::Pose2d targetPose) 
 : m_swerve{swerve}, m_vision{vision}, m_targetPose{targetPose} {
  SetName( "ProfiledDriveToPose" );
  // Use addRequirements() here to declare subsystem dependencies.
  AddRequirements({swerve});
}

// Called when the command is initially scheduled.
void ProfiledDriveToPose::Init() {
  // m_xSetpoint.position = m_vision->GetRelativePose().X();
  // m_ySetpoint.position = m_vision->GetRelativePose().Y();
  // m_omegaSetpoint.position = m_vision->GetRelativePose().Rotation().Degrees();

  m_xSetpoint.position = m_swerve->GetPose().X();
  m_ySetpoint.position = m_swerve->GetPose().Y();
  m_omegaSetpoint.position = m_swerve->GetPose().Rotation().Degrees();

  // m_xSetpoint.position = 0_m;
  // m_ySetpoint.position = 0_m;
  // m_omegaSetpoint.position = 0_deg;

  m_xGoal.position = m_targetPose.X();
  m_xGoal.velocity = 0_mps;
  m_yGoal.position = m_targetPose.Y();
  m_yGoal.velocity = 0_mps;
  m_omegaGoal.position = m_targetPose.Rotation().Degrees();
  m_omegaGoal.velocity = 0_rpm;

  m_startTime = frc::Timer::GetFPGATimestamp();

  DataLogger::GetInstance().SendNT( "Profiled Drive/xGoal",m_xGoal.position.value() );
  DataLogger::GetInstance().SendNT( "Profiled Drive/yGoal",m_yGoal.position.value() );
  DataLogger::GetInstance().SendNT( "Profiled Drive/omegaGoal",m_omegaGoal.position.value() );

  // fmt::print( "Initial profile lengths : {} {} {}\n", m_xProfile.TotalTime(), m_yProfile.TotalTime(), m_omegaProfile.TotalTime());
}

// Called repeatedly when this Command is scheduled to run
void ProfiledDriveToPose::Execute() {
  m_xSetpoint.position = m_swerve->GetPose().X();
  m_ySetpoint.position = m_swerve->GetPose().Y();
  m_omegaSetpoint.position = m_swerve->GetPose().Rotation().Degrees();

  if(m_omegaGoal.position - m_omegaSetpoint.position > 180_deg) {
    m_omegaSetpoint.position += 360_deg;
  } else if(m_omegaGoal.position - m_omegaSetpoint.position < -180_deg) {
    m_omegaSetpoint.position -= 360_deg;
  }

  

  m_xSetpoint = m_xProfile.Calculate(20_ms, m_xSetpoint, m_xGoal);
  m_ySetpoint = m_yProfile.Calculate(20_ms, m_ySetpoint, m_yGoal);
  m_omegaSetpoint = m_omegaProfile.Calculate(20_ms, m_omegaSetpoint, m_omegaGoal);

  frc::SmartDashboard::PutNumber( "ProfileX", m_xSetpoint.position.value() );
  frc::SmartDashboard::PutNumber( "ProfileY", m_ySetpoint.position.value() );
  frc::SmartDashboard::PutNumber( "ProfileVX", m_xSetpoint.velocity.value() );
  frc::SmartDashboard::PutNumber( "ProfileVY", m_ySetpoint.velocity.value() );
  DataLogger::GetInstance().SendNT( "Profiled Drive/Pose",m_swerve->GetPose() );

  // fmt::print( "Profile lengths : {} {} {}\n", m_xProfile.TotalTime(), m_yProfile.TotalTime(), m_omegaProfile.TotalTime());

  frc::ChassisSpeeds speeds;
  if(units::math::abs(m_swerve->GetPose().X() - m_xGoal.position) > 0.015_m) {
    speeds.vx = m_xSetpoint.velocity;
  }
  if(units::math::abs(m_swerve->GetPose().Y() - m_yGoal.position) > 0.015_m) {
    speeds.vy = m_ySetpoint.velocity;
  }
  if(units::math::abs(m_swerve->GetPose().Rotation().Degrees() - m_omegaGoal.position) > 3_deg) {
    speeds.omega = m_omegaSetpoint.velocity;
  }



  m_swerve->Drive( speeds, true );
}

// Called once the command ends or is interrupted.
void ProfiledDriveToPose::Ending(bool interrupted) {
  fmt::print( "   ProfiledDriveToPose::End() interrupted {}\n", interrupted );
}

// Returns true when the command should end.
bool ProfiledDriveToPose::IsFinished() {

      // NOTE::
      //
      //   IsFinished() returned true at the halfway point of the trajectory.
      //   Changed to use TotalTime() which goes to zero at the end of the trajectory.
      //
//  units::second_t m_elapsed = frc::Timer::GetFPGATimestamp() - m_startTime;
// fmt::print( " IsFinished at {} : {} {} {}\n", m_elapsed, m_xProfile.IsFinished(m_elapsed), m_yProfile.IsFinished(m_elapsed), m_omegaProfile.IsFinished(m_elapsed));
  // bool atTargetLocation = m_xProfile.IsFinished(m_elapsed) && m_yProfile.IsFinished(m_elapsed) && m_omegaProfile.IsFinished(m_elapsed);
  bool atTargetLocation = units::math::abs(m_swerve->GetPose().X() - m_xGoal.position) < 0.015_m 
                          && units::math::abs(m_swerve->GetPose().Y() - m_yGoal.position) < 0.015_m 
                          && units::math::abs(m_swerve->GetPose().Rotation().Degrees() - m_omegaGoal.position) < 3_deg;
  return atTargetLocation;
}

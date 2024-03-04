// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "commands/ProfiledDriveToPose.h"

ProfiledDriveToPose::ProfiledDriveToPose(SwerveDriveSubsystem *swerve, frc::Pose2d targetPose) 
 : m_swerve{swerve}, m_targetPose{targetPose} {
  // Use addRequirements() here to declare subsystem dependencies.
  AddRequirements({swerve});
}

// Called when the command is initially scheduled.
void ProfiledDriveToPose::Initialize() {
  m_xSetpoint.position = 0.0_m;
  m_ySetpoint.position = 0.0_m;
  m_omegaSetpoint.position = 0_deg;

  m_xGoal.position = m_targetPose.X();
  m_yGoal.position = m_targetPose.Y();
  m_omegaGoal.position = m_targetPose.Rotation().Degrees();

  m_startTime = frc::Timer::GetFPGATimestamp();
}

// Called repeatedly when this Command is scheduled to run
void ProfiledDriveToPose::Execute() {
  m_xSetpoint = m_xProfile.Calculate(20_ms, m_xSetpoint, m_xGoal);
  m_ySetpoint = m_yProfile.Calculate(20_ms, m_ySetpoint, m_yGoal);
  m_omegaSetpoint = m_omegaProfile.Calculate(20_ms, m_omegaSetpoint, m_omegaGoal);

  frc::ChassisSpeeds speeds;
  speeds.vx = m_xSetpoint.velocity;
  speeds.vy = m_ySetpoint.velocity;
  speeds.omega = m_omegaSetpoint.velocity;

  m_swerve->Drive(speeds);
}

// Called once the command ends or is interrupted.
void ProfiledDriveToPose::End(bool interrupted) {}

// Returns true when the command should end.
bool ProfiledDriveToPose::IsFinished() {
  units::second_t m_elapsed = frc::Timer::GetFPGATimestamp() - m_startTime;
  bool atTargetLocation = m_xProfile.IsFinished(m_elapsed) && m_yProfile.IsFinished(m_elapsed) && m_omegaProfile.IsFinished(m_elapsed);
  return atTargetLocation;
}

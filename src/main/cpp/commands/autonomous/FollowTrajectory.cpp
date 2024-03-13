// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "DataLogger.h"
#include "commands/autonomous/FollowTrajectory.h"

#include <frc/smartdashboard/SmartDashboard.h>

FollowTrajectory::FollowTrajectory(SwerveDriveSubsystem *swerve, frc::Trajectory trajectory, frc::Rotation2d endAngle)
 : m_swerve{swerve}, m_trajectory{trajectory}, m_endAngle{endAngle} {
  // Use addRequirements() here to declare subsystem dependencies.
  AddRequirements({swerve});
}

// Called when the command is initially scheduled.
void FollowTrajectory::Initialize() {
  DataLogger::GetInstance().Send( "Command/FollowTrajectory", true );

  m_swerve->m_field.GetObject("trajectory")->SetTrajectory(m_trajectory);
  m_autoElapsed = 0_ms;
}

// Called repeatedly when this Command is scheduled to run
void FollowTrajectory::Execute() {
  auto goal = m_trajectory.Sample(m_autoElapsed);

  frc::SmartDashboard::PutNumber("Goal X", goal.pose.X().value());
  frc::SmartDashboard::PutNumber("Goal Y", goal.pose.Y().value());
  frc::SmartDashboard::PutNumber("End Angle", m_endAngle.Degrees().value());

  m_swerve->DriveTrajectory(goal, m_endAngle);

  m_autoElapsed += 20_ms;
}

// Called once the command ends or is interrupted.
void FollowTrajectory::End(bool interrupted) {
    DataLogger::GetInstance().Send( "Command/FollowTrajectory", false );
}

// Returns true when the command should end.
bool FollowTrajectory::IsFinished() {
  atPosition = m_trajectory.TotalTime() < m_autoElapsed && units::math::abs(m_endAngle.Degrees() - m_swerve->GetPose().Rotation().Degrees()) < 1_deg;
  if (atPosition) {
    fmt::print("FollowTrajectory::IsFinished()\n");
  }
  return atPosition;
}

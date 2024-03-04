// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "commands/ChangeShooterAngle.h"

ChangeShooterAngle::ChangeShooterAngle(ShooterSubsystem* shooter, units::degree_t angle) 
 : m_shooter{shooter}, m_angle{angle} {
  // Use addRequirements() here to declare subsystem dependencies.
  AddRequirements({shooter});
}

// Called when the command is initially scheduled.
void ChangeShooterAngle::Initialize() {}

// Called repeatedly when this Command is scheduled to run
void ChangeShooterAngle::Execute() {
  m_shooter->GoToAngle(m_angle);
}

// Called once the command ends or is interrupted.
void ChangeShooterAngle::End(bool interrupted) {}

// Returns true when the command should end.
bool ChangeShooterAngle::IsFinished() {
  return (units::math::abs(m_angle - m_shooter->GetShooterAngle()) < 3_deg);
}

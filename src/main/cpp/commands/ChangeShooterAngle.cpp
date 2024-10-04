// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "subsystems/ShooterSubsystem.h"

#include "commands/ChangeShooterAngle.h"

ChangeShooterAngle::ChangeShooterAngle(ShooterSubsystem* shooter, units::degree_t angle) 
 : m_shooter{shooter}, m_angle{angle} {
  SetName( "ChangeShooterAngle" );
  // Use addRequirements() here to declare subsystem dependencies.
  AddRequirements({shooter});
}

// Called when the command is initially scheduled.
void ChangeShooterAngle::Init() {
}

// Called repeatedly when this Command is scheduled to run
void ChangeShooterAngle::Execute() {
  m_shooter->GoToAngle(m_angle);
}

// Returns true when the command should end.
bool ChangeShooterAngle::IsFinished() {
  return m_shooter->IsAtGoal();
}

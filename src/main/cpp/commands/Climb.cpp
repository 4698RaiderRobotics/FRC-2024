// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "DataLogger.h"

#include "commands/Climb.h"

Climb::Climb(ClimberSubsystem *climber)
 : m_climber{climber} {
  // Use addRequirements() here to declare subsystem dependencies.
  AddRequirements({climber});
}

// Called when the command is initially scheduled.
void Climb::Initialize() {
  DataLogger::GetInstance().Send( "Command/Climb", true );
  m_climber->SetSpeed(-0.75);
}

// Called repeatedly when this Command is scheduled to run
void Climb::Execute() {}

// Called once the command ends or is interrupted.
void Climb::End(bool interrupted) {
  m_climber->SetSpeed(0.0);
  DataLogger::GetInstance().Send( "Command/Climb", false );
}

// Returns true when the command should end.
bool Climb::IsFinished() {
  return m_climber->AtLimit();
}

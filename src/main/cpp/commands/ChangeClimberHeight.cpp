// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "DataLogger.h"

#include "commands/ChangeClimberHeight.h"

ChangeClimberHeight::ChangeClimberHeight(ClimberSubsystem *climber, double rotations) 
 : m_climber{climber}, m_rotations{rotations} {
  // Use addRequirements() here to declare subsystem dependencies.
  AddRequirements({climber});
}

// Called when the command is initially scheduled.
void ChangeClimberHeight::Initialize() {
  DataLogger::GetInstance().Send( "Command/ChangeClimberHeight", true );

  if(m_rotations - m_climber->GetRotations() > 0.0) {
    m_climber->SetSpeed(0.5);
  } else {
    m_climber->SetSpeed(-0.5);
  }
}

// Called repeatedly when this Command is scheduled to run
void ChangeClimberHeight::Execute() {}

// Called once the command ends or is interrupted.
void ChangeClimberHeight::End(bool interrupted) {
  // fmt::print("ChangeClimberHeight::IsFinished()");
  m_climber->SetSpeed(0.0);
  DataLogger::GetInstance().Send( "Command/ChangeClimberHeight", false );
}

// Returns true when the command should end.
bool ChangeClimberHeight::IsFinished() {
  return std::abs(m_climber->GetRotations() - m_rotations) < 5;
}

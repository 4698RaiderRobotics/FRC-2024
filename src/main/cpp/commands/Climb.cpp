// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "subsystems/ClimberSubsystem.h"

#include "commands/Climb.h"

Climb::Climb(ClimberSubsystem *climber, units::inch_t height_target )
 : m_climber{climber}, height_target{height_target} {
  SetName( "Climb" );
  // Use addRequirements() here to declare subsystem dependencies.
  AddRequirements({climber});
}


// Called repeatedly when this Command is scheduled to run
void Climb::Execute() {
  m_climber->GoToHeight(height_target);
}

// Returns true when the command should end.
bool Climb::IsFinished() {
  return m_climber->IsAtGoal();
}

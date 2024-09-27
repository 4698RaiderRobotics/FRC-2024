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

// Called when the command is initially scheduled.
void Climb::Init() {
  if( height_target < m_climber->GetHeight() ) {
    m_climber->SetSpeed(-1);
  } else {
    m_climber->SetSpeed(1);
  }
}

// Called repeatedly when this Command is scheduled to run
void Climb::Execute() {}

// Called once the command ends or is interrupted.
void Climb::Ending(bool interrupted) {
  m_climber->SetSpeed(0.0);
}

// Returns true when the command should end.
bool Climb::IsFinished() {

    // Stop if the limit switch is tripped.
  if( m_climber->AtLimit() ) {
    return true;
  }
  
  units::inch_t height_error = units::math::abs( m_climber->GetHeight() - height_target );

      // Within 0.25 inch is at target
  return height_error < 0.25_in;
}

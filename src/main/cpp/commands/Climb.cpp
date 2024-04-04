// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "commands/Climb.h"

Climb::Climb(ClimberSubsystem *climber, double rotation_target )
 : m_climber{climber}, rot_target{rotation_target} {
  SetName( "Climb" );
  // Use addRequirements() here to declare subsystem dependencies.
  AddRequirements({climber});
}

// Called when the command is initially scheduled.
void Climb::Init() {
  if( rot_target < m_climber->GetRotations() ) {
    going_up = false;
    m_climber->SetSpeed(-1);
  } else {
    going_up = true;
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
  bool at_rotations = false; 

  if( m_climber->AtLimit() ) {
    return true;
  }

  double rotations_left = m_climber->GetRotations() - rot_target;
  if( going_up ) {
    if( rotations_left > 0.0 ) {
      // Climber is at target
      return true;
    }
  } else {
    if( rotations_left <  0.0 ) {
      return true;
    }
  }

  return false;
}

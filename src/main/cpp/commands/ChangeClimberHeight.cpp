// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "subsystems/ClimberSubsystem.h"

#include "commands/ChangeClimberHeight.h"

ChangeClimberHeight::ChangeClimberHeight(ClimberSubsystem *climber, units::inch_t target_height) 
 : m_climber{climber}, m_target_height{target_height} {
  SetName( "ChangeClimberHeight" );
  // Use addRequirements() here to declare subsystem dependencies.
  AddRequirements({climber});
}

// Called when the command is initially scheduled.
void ChangeClimberHeight::Init() {

  if(m_target_height - m_climber->GetHeight() > 0.0_in) {
    m_climber->SetSpeed(0.5);
    going_up = true;
  } else {
    m_climber->SetSpeed(-0.5);
    going_up = false;
  }
}

// Called repeatedly when this Command is scheduled to run
void ChangeClimberHeight::Execute() {}

// Called once the command ends or is interrupted.
void ChangeClimberHeight::Ending(bool interrupted) {
  m_climber->SetSpeed(0.0);
}

// Returns true when the command should end.
bool ChangeClimberHeight::IsFinished() {
  if( m_climber->AtLimit() ) {
    return true;
  }
  
  units::inch_t height_error = m_climber->GetHeight() - m_target_height;
  if( going_up ) {
    if( height_error > 0.0_in ) {
      // Climber is at target
      return true;
    }
  } else {
    if( height_error <  0.0_in ) {
      return true;
    }
  }

  return false;
}

// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "subsystems/ElevatorSubsystem.h"

#include "commands/ChangeElevatorHeight.h"

ChangeElevatorHeight::ChangeElevatorHeight(ElevatorSubsystem* elevator, units::meter_t height)
 : m_elevator{elevator}, m_height{height} {
  SetName( "ChangeElevatorHeight" );
  // Use addRequirements() here to declare subsystem dependencies.
  AddRequirements({elevator});
}

// Called when the command is initially scheduled.
void ChangeElevatorHeight::Init() {
}

// Called repeatedly when this Command is scheduled to run
void ChangeElevatorHeight::Execute() {
  m_elevator->GoToHeight(m_height);
}

// Called once the command ends or is interrupted.
void ChangeElevatorHeight::Ending(bool interrupted) {
}

// Returns true when the command should end.
bool ChangeElevatorHeight::IsFinished() {
  return m_elevator->IsAtGoal();
}

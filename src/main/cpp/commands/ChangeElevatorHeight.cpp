// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "commands/ChangeElevatorHeight.h"

ChangeElevatorHeight::ChangeElevatorHeight(ElevatorSubsystem* elevator, units::meter_t height)
 : m_elevator{elevator}, m_height{height} {
  // Use addRequirements() here to declare subsystem dependencies.
  AddRequirements({elevator});
}

// Called when the command is initially scheduled.
void ChangeElevatorHeight::Initialize() {}

// Called repeatedly when this Command is scheduled to run
void ChangeElevatorHeight::Execute() {
  m_elevator->GoToHeight(m_height);
}

// Called once the command ends or is interrupted.
void ChangeElevatorHeight::End(bool interrupted) {}

// Returns true when the command should end.
bool ChangeElevatorHeight::IsFinished() {
  return units::math::abs(m_height - m_elevator->GetHeight()) < 0.5_in;
}

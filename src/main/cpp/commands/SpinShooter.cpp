// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "commands/SpinShooter.h"

SpinShooter::SpinShooter(ShooterSubsystem* shooter, double speed)
 : m_shooter{shooter}, m_speed{speed} {
  // Use addRequirements() here to declare subsystem dependencies.
  AddRequirements({shooter});
}

// Called when the command is initially scheduled.
void SpinShooter::Initialize() {}

// Called repeatedly when this Command is scheduled to run
void SpinShooter::Execute() {
  m_shooter->Spin(m_speed);
}

// Called once the command ends or is interrupted.
void SpinShooter::End(bool interrupted) {}

// Returns true when the command should end.
bool SpinShooter::IsFinished() {
  return m_shooter->AtSpeed();
}

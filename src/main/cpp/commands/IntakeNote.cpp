// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "commands/IntakeNote.h"

#include <frc/Timer.h>

IntakeNote::IntakeNote(IntakeSubsystem* intake)
 : m_intake{intake} {
  // Use addRequirements() here to declare subsystem dependencies.
  AddRequirements({intake});
}

// Called when the command is initially scheduled.
void IntakeNote::Initialize() {
  m_intake->SpinIntake(physical::kIntakeSpeed);
  m_startTime = frc::Timer::GetFPGATimestamp();
}

// Called repeatedly when this Command is scheduled to run
void IntakeNote::Execute() {}

// Called once the command ends or is interrupted.
void IntakeNote::End(bool interrupted) {
  m_intake->hasNote = true;
  m_intake->SpinIntake(0.0);
}

// Returns true when the command should end.
bool IntakeNote::IsFinished() {
  return frc::Timer::GetFPGATimestamp() - m_startTime > 3_s;
}

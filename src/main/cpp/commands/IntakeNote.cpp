// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include <frc/Timer.h>

#include "commands/IntakeNote.h"

IntakeNote::IntakeNote(IntakeSubsystem* intake)
 : m_intake{intake} {
  SetName( "IntakeNote" );
  // Use addRequirements() here to declare subsystem dependencies.
  AddRequirements({intake});
}

// Called when the command is initially scheduled.
void IntakeNote::Init() {

  m_intake->SpinIntake(physical::kIntakeSpeed);
  m_startTime = frc::Timer::GetFPGATimestamp();
  beamHasBroken = false;
  isFinished = false;
}

// Called repeatedly when this Command is scheduled to run
void IntakeNote::Execute() {
  if (!beamHasBroken && m_intake->IsBeamBroken()) {
    beamHasBroken = true;
    isFinished = true;
  }
  // fmt::print( "   IntakeNote::Execute beam broken {}, isFinished {}\n", m_intake->IsBeamBroken(), isFinished );
}

// Called once the command ends or is interrupted.
void IntakeNote::HasEnded(bool interrupted) {
  // fmt::print( "   IntakeNote::End interrupted {}\n", interrupted );
  if(!interrupted) {
    m_intake->centering = true;
    m_intake->SpinIntake(0.0);
  }
}

// Returns true when the command should end.
bool IntakeNote::IsFinished() {
  return frc::Timer::GetFPGATimestamp() - m_startTime > 10_s || isFinished;
}

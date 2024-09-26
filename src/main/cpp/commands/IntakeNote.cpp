// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include <frc/Timer.h>

#include "DataLogger.h"

#include "subsystems/IntakeSubsystem.h"

#include "commands/IntakeNote.h"

IntakeNote::IntakeNote(IntakeSubsystem* intake)
 : m_intake{intake} {
  SetName( "IntakeNote" );
  // Use addRequirements() here to declare subsystem dependencies.
  AddRequirements({intake});
}

// Called when the command is initially scheduled.
void IntakeNote::Init() {

  m_intake->SpinIntake(0.8);
  m_startTime = frc::Timer::GetFPGATimestamp();
  beamHasBroken = false;

  DataLogger::GetInstance().SendNT( "IntakeNote/beamHasBroken", beamHasBroken );
}

// Called repeatedly when this Command is scheduled to run
void IntakeNote::Execute() {
  if ( /* !beamHasBroken && */ m_intake->IsBeamBroken()) {
    beamHasBroken = true;
    m_intake->NotePickedUp();
  }
  DataLogger::GetInstance().SendNT( "IntakeNote/beamHasBroken", beamHasBroken );
}

// Called once the command ends or is interrupted.
void IntakeNote::Ending(bool interrupted) {
  m_intake->SpinIntake(0.0);
  if( m_intake->IsBeamBroken()) {
    m_intake->NotePickedUp();
  }
}

// Returns true when the command should end.
bool IntakeNote::IsFinished() {
  return frc::Timer::GetFPGATimestamp() - m_startTime > 10_s || beamHasBroken;
}

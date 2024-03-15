// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "commands/SpinShooter.h"

#include <frc/Timer.h>

SpinShooter::SpinShooter(ShooterSubsystem* shooter, units::revolutions_per_minute_t speed)
 : m_shooter{shooter}, m_speed{speed} {
  SetName( "SpinShooter" );
  // Use addRequirements() here to declare subsystem dependencies
  AddRequirements({shooter});
}

// Called when the command is initially scheduled.
void SpinShooter::Init() {
  m_startTime = frc::Timer::GetFPGATimestamp();
}

// Called repeatedly when this Command is scheduled to run
void SpinShooter::Execute() {
  m_shooter->Spin(m_speed);
  // fmt::print("ShooterAtSpeed: {}\n", m_shooter->IsAtSpeed());
}

// Called once the command ends or is interrupted.
void SpinShooter::Ending(bool interrupted) {
}

// Returns true when the command should end.
bool SpinShooter::IsFinished() {
  return m_shooter->IsAtSpeed() || frc::Timer::GetFPGATimestamp() - m_startTime > 4_s;
}

// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "subsystems/ArmSubsystem.h"

#include "commands/ChangeWristAngle.h"

ChangeWristAngle::ChangeWristAngle(ArmSubsystem* arm, units::degree_t wristAngle)
 : m_arm{arm}, m_wristAngle{wristAngle} {
  SetName( "ChangeWristAngle" );
  // Use addRequirements() here to declare subsystem dependencies.
  AddRequirements({arm});
}

// Called when the command is initially scheduled.
void ChangeWristAngle::Init() {
}

// Called repeatedly when this Command is scheduled to run
void ChangeWristAngle::Execute() {
  m_arm->GoToWristAngle(m_wristAngle);
}

// Returns true when the command should end.
bool ChangeWristAngle::IsFinished() {
  bool done = units::math::abs(m_wristAngle - m_arm->GetWristAngle()) < 5_deg;
  return done;
}

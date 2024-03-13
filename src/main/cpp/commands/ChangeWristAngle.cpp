// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "DataLogger.h"
#include "commands/ChangeWristAngle.h"

ChangeWristAngle::ChangeWristAngle(ArmSubsystem* arm, units::degree_t wristAngle)
 : m_arm{arm}, m_wristAngle{wristAngle} {
  // Use addRequirements() here to declare subsystem dependencies.
  AddRequirements({arm});
}

// Called when the command is initially scheduled.
void ChangeWristAngle::Initialize() {
      DataLogger::GetInstance().Send( "Command/ChangeWristAngle", true );
}

// Called repeatedly when this Command is scheduled to run
void ChangeWristAngle::Execute() {
  m_arm->GoToWristAngle(m_wristAngle);
}

// Called once the command ends or is interrupted.
void ChangeWristAngle::End(bool interrupted) {
        DataLogger::GetInstance().Send( "Command/ChangeWristAngle", false );
}

// Returns true when the command should end.
bool ChangeWristAngle::IsFinished() {
  bool done = units::math::abs(m_wristAngle - m_arm->GetWristAngle()) < 5_deg;
  return done;
}

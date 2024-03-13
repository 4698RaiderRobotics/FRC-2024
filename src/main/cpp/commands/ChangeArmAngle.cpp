// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "DataLogger.h"
#include "commands/ChangeArmAngle.h"

ChangeArmAngle::ChangeArmAngle(ArmSubsystem* arm, units::degree_t armAngle)
 : m_arm{arm}, m_armAngle{armAngle} {
  // Use addRequirements() here to declare subsystem dependencies.
  AddRequirements({arm});
}

// Called when the command is initially scheduled.
void ChangeArmAngle::Initialize() {
    DataLogger::GetInstance().Send( "Command/ChangeArmAngle", true );
}

// Called repeatedly when this Command is scheduled to run
void ChangeArmAngle::Execute() {
  m_arm->GoToArmAngle(m_armAngle);
}

// Called once the command ends or is interrupted.
void ChangeArmAngle::End(bool interrupted) {
      DataLogger::GetInstance().Send( "Command/ChangeArmAngle", false );
}

// Returns true when the command should end.
bool ChangeArmAngle::IsFinished() {
  return true;
}

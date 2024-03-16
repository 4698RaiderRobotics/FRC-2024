// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "commands/ChangeArmAngle.h"

ChangeArmAngle::ChangeArmAngle(ArmSubsystem* arm, units::degree_t armAngle)
 : m_arm{arm}, m_armAngle{armAngle} {
  SetName( "ChangeArmAngle" );
  // Use addRequirements() here to declare subsystem dependencies.
  AddRequirements({arm});
}

// Called repeatedly when this Command is scheduled to run
void ChangeArmAngle::Execute() {
  m_arm->GoToArmAngle(m_armAngle);
}

// Returns true when the command should end.
bool ChangeArmAngle::IsFinished() {
  return true;
}

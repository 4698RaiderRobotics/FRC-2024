// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "commands/ChangeArmAngle.h"

ChangeArmAngle::ChangeArmAngle(ArmSubsystem* arm, units::degree_t armAngle, units::degree_t wristAngle)
 : m_arm{arm}, m_armAngle{armAngle}, m_wristAngle{wristAngle} {
  // Use addRequirements() here to declare subsystem dependencies.
  AddRequirements({arm});
}

// Called when the command is initially scheduled.
void ChangeArmAngle::Initialize() {}

// Called repeatedly when this Command is scheduled to run
void ChangeArmAngle::Execute() {
  m_arm->GoToAngle(m_armAngle, m_wristAngle);
}

// Called once the command ends or is interrupted.
void ChangeArmAngle::End(bool interrupted) {}

// Returns true when the command should end.
bool ChangeArmAngle::IsFinished() {
  return true;
}

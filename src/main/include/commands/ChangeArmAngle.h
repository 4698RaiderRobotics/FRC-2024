// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc2/command/Command.h>
#include <frc2/command/CommandHelper.h>

#include "LoggedCommand.h"
#include "subsystems/ArmSubsystem.h"

/**
 * Change the Arm Angle Set Point and return immediately.
 */
class ChangeArmAngle
    : public frc2::CommandHelper<LoggedCommand, ChangeArmAngle> {
 public:
  ChangeArmAngle(ArmSubsystem* arm, units::degree_t armAngle);

  void Execute() override;

  bool IsFinished() override;
 
 private:
  ArmSubsystem* m_arm;
  units::degree_t m_armAngle;
};

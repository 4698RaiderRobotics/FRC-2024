// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "RobotContainer.h"

#include <frc2/command/Commands.h>
#include <frc2/command/RunCommand.h>

#include "commands/SpinShooter.h"

RobotContainer::RobotContainer() {
  m_swerveDrive.SetDefaultCommand(frc2::RunCommand(
    [this] {
      m_swerveDrive.ArcadeDrive(vx_axis.GetAxis(), vy_axis.GetAxis(), omega_axis.GetAxis());
   },
    { &m_swerveDrive }
    ));

  ConfigureBindings();
}

void RobotContainer::ConfigureBindings() {
  m_operatorController.A().OnTrue(SpinShooter(&m_shooter, 0.4).ToPtr()).OnFalse(SpinShooter(&m_shooter, 0.0).ToPtr());
}

frc2::CommandPtr RobotContainer::GetAutonomousCommand() {
  return frc2::cmd::Print("No autonomous command configured");
}

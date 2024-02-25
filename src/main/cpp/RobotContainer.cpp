// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "RobotContainer.h"

#include <frc2/command/Commands.h>
#include <frc2/command/RunCommand.h>
#include <frc2/command/SequentialCommandGroup.h>

#include "commands/SpinShooter.h"
#include "commands/ChangeShooterAngle.h"
#include "commands/ChangeArmAngle.h"
#include "commands/ChangeWristAngle.h"
#include "commands/IntakeNote.h"

RobotContainer::RobotContainer() 
: m_swerveDrive{&m_vision} {
  m_swerveDrive.SetDefaultCommand(frc2::RunCommand(
    [this] {
      m_swerveDrive.ArcadeDrive(vx_axis.GetAxis(), vy_axis.GetAxis(), omega_axis.GetAxis());
   },
    { &m_swerveDrive }
    ));

  ConfigureBindings();
}

void RobotContainer::ConfigureBindings() {
  // Spins up the shooter while the A button is held
  m_operatorController.A().OnTrue(SpinShooter(&m_shooter, 0.4).ToPtr()).OnFalse(SpinShooter(&m_shooter, 0.0).ToPtr());

  // Sets the shooter angle to 30 when the B button is pressed
  m_operatorController.B().OnTrue(ChangeShooterAngle(&m_shooter, 30_deg).ToPtr());

  // Sets the shooter angle to 60 when the X button is pressed
  m_operatorController.X().OnTrue(ChangeShooterAngle(&m_shooter, 60_deg).ToPtr());

  m_operatorController.RightBumper().OnTrue(frc2::SequentialCommandGroup(ChangeArmAngle(&m_arm, 10_deg), ChangeWristAngle(&m_arm, -60_deg)).ToPtr());

  m_operatorController.LeftBumper().OnTrue(frc2::SequentialCommandGroup(ChangeArmAngle(&m_arm, 135_deg), ChangeWristAngle(&m_arm, 0_deg)).ToPtr());

  m_operatorController.RightTrigger().OnTrue(frc2::SequentialCommandGroup(ChangeArmAngle(&m_arm, 135_deg), ChangeWristAngle(&m_arm, 170_deg)).ToPtr());

  m_operatorController.LeftTrigger().OnTrue(IntakeNote(&m_intake).ToPtr());
}

frc2::CommandPtr RobotContainer::GetAutonomousCommand() {
  return frc2::cmd::Print("No autonomous command configured");
}

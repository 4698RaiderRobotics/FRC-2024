// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc2/command/CommandPtr.h>

#include <frc/XboxController.h>
#include <frc/PS5Controller.h>

#include "ControllerAxis.h"
#include "subsystems/SwerveDriveSubsystem.h"


class RobotContainer {
 public:
  RobotContainer();

  frc2::CommandPtr GetAutonomousCommand();

 private:
  void ConfigureBindings();

  frc::PS5Controller m_driverController{0};
  frc::XboxController m_operatorController{1};

  ControllerAxis vx_axis{m_driverController, frc::PS5Controller::Axis::kLeftY, true};
  ControllerAxis vy_axis{m_driverController, frc::PS5Controller::Axis::kLeftX, true};
  ControllerAxis omega_axis{m_driverController, frc::PS5Controller::Axis::kRightX, true};

  SwerveDriveSubsystem m_swerveDrive;
};

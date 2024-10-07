// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc2/command/CommandPtr.h>

#include <frc2/command/button/CommandPS5Controller.h>
#include <frc2/command/button/CommandXboxController.h>
#include <frc/smartdashboard/SendableChooser.h>

#include "ControllerAxis.h"
#include "subsystems/SwerveDriveSubsystem.h"
#include "subsystems/ArmSubsystem.h"
#include "subsystems/ClimberSubsystem.h"
#include "subsystems/ElevatorSubsystem.h"
#include "subsystems/IntakeSubsystem.h"
#include "subsystems/ShooterSubsystem.h"
#include "subsystems/VisionSubsystem.h"
#include "subsystems/LEDSubsystem.h"

#include "commands/ComplexCmds.h"


class RobotContainer {
 public:
  RobotContainer();

  frc2::Command* GetAutonomousCommand();

 private:
  void ConfigureBindings();
  void ConfigureAutos();

  frc::SendableChooser<int> m_chooser;

  std::vector<frc2::CommandPtr> AutoCommands;
  
  struct AutoNameMap {
    std::string Description;
    std::string AutoName;
  };

  LEDSubsystem m_leds;
  SwerveDriveSubsystem m_swerveDrive;
  ElevatorSubsystem m_elevator;
  IntakeSubsystem m_intake;
  ClimberSubsystem m_climber;
  VisionSubsystem m_vision;

  frc2::CommandPS5Controller m_driverController{0};
  frc2::CommandXboxController m_operatorController{1};

  AutoClimbAndTrap m_climbAndTrapCmd{ &m_swerveDrive, &m_intake, &m_arm,
                         &m_elevator, &m_climber, &m_shooter, &m_vision};
  MoveToAndPlaceInAmp m_moveToAmpCmd{ &m_swerveDrive, &m_intake, &m_arm,
                         &m_elevator, &m_vision};

  ControllerAxis vx_axis{m_driverController, frc::PS5Controller::Axis::kLeftY, true};
  ControllerAxis vy_axis{m_driverController, frc::PS5Controller::Axis::kLeftX, true};
  ControllerAxis omega_axis{m_driverController, frc::PS5Controller::Axis::kRightX, true};
  ControllerAxis arm_angle_axis{ m_operatorController, frc::XboxController::Axis::kLeftY, true };
  ControllerAxis wrist_angle_axis{ m_operatorController, frc::XboxController::Axis::kRightY, true };
  ControllerAxis elevator_axis{ m_operatorController, frc::XboxController::Axis::kLeftX, true };

public:
    // Subsystems that need offsets stored in preferences...
  ArmSubsystem m_arm;
  ShooterSubsystem m_shooter;
};

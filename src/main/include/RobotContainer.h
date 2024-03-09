// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc2/command/CommandPtr.h>

#include <frc/PS5Controller.h>
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


class RobotContainer {
 public:
  RobotContainer();

  frc2::Command* GetAutonomousCommand();

 private:
  void ConfigureBindings();

  frc2::Command* m_autoCommand = nullptr;

  frc::SendableChooser<std::string> m_chooser;
  const std::string kTwoPieceMiddle = "Two Piece Middle Auto";
  const std::string kOnePiece = "One Piece Auto";
  const std::string kTwoPieceLeft = "Two Piece Left Auto";
  const std::string kTwoPieceRight = "Two Piece Right Auto";
  const std::string kOnePieceTaxi = "One Piece Taxi Auto";
  std::string m_autoSelected;

  LEDSubsystem m_leds;
  SwerveDriveSubsystem m_swerveDrive;
  ArmSubsystem m_arm;
  ClimberSubsystem m_climber;
  ElevatorSubsystem m_elevator;
  IntakeSubsystem m_intake;
  ShooterSubsystem m_shooter;
  VisionSubsystem m_vision;

  frc::PS5Controller m_driverController{0};
  frc2::CommandXboxController m_operatorController{1};

  ControllerAxis vx_axis{m_driverController, frc::PS5Controller::Axis::kLeftY, true};
  ControllerAxis vy_axis{m_driverController, frc::PS5Controller::Axis::kLeftX, true};
  ControllerAxis omega_axis{m_driverController, frc::PS5Controller::Axis::kRightX, true};
  ControllerAxis arm_angle_axis{ m_operatorController, frc::XboxController::Axis::kLeftY, true };
  ControllerAxis wrist_angle_axis{ m_operatorController, frc::XboxController::Axis::kRightY, true };
  ControllerAxis elevator_axis{ m_operatorController, frc::XboxController::Axis::kLeftX, true };

};

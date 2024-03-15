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


class RobotContainer {
 public:
  RobotContainer();

  frc2::Command* GetAutonomousCommand();

 private:
  void ConfigureBindings();

  frc::SendableChooser<frc2::Command*> m_chooser;
  const std::string kOnePiece = "One Piece";

  const std::string kSourceFourPiece = "Source Four Piece";
  const std::string kSourceThreePiece = "Source Three Piece";
  const std::string kSourceTwoPiece = "Source Two Piece";
  const std::string kSourceTwoPieceCenter = "Source Two Piece Center";
  const std::string kSourceThreePieceCenter = "Source Three Piece Center";
  const std::string kSourceFourPieceCenter = "Source Four Piece Center";
  const std::string kSourceOnePieceTaxi = "Source One Piece Taxi";

  const std::string kAmpFourPiece = "Amp Four Piece";
  const std::string kAmpThreePiece = "Amp Three Piece";
  const std::string kAmpTwoPiece = "Amp Two Piece";
  const std::string kAmpThreePieceCenter = "Amp Three Piece Center";
  const std::string kAmpFourPieceCenter = "Amp Four Piece Center";

  const std::string kMiddleFourPiece = "Middle Four Piece";
  const std::string kMiddleThreePieceAmp = "Middle Three Piece Amp";
  const std::string kMiddleThreePieceSource = "Middle Three Piece Source";
  const std::string kMiddleTwoPiece = "Middle Two Piece";
  const std::string kMiddleFourPieceCenterAmp = "Middle Four Piece Center Amp";
  const std::string kMiddleFourPieceCenterSource = "Middle Four Piece Center Source";


  // const std::string kTwoPieceMiddle = "Two Piece Middle Auto";
  // const std::string kOnePiece = "One Piece Auto";
  // const std::string kTwoPieceLeft = "Two Piece Left Auto";
  // const std::string kTwoPieceRight = "Two Piece Right Auto";
  // const std::string kOnePieceTaxi = "One Piece Taxi Auto";
  // std::string m_autoSelected;

  LEDSubsystem m_leds;
  SwerveDriveSubsystem m_swerveDrive;
  ArmSubsystem m_arm;
  ClimberSubsystem m_climber;
  ElevatorSubsystem m_elevator;
  IntakeSubsystem m_intake;
  ShooterSubsystem m_shooter;
  VisionSubsystem m_vision;

  frc2::CommandPS5Controller m_driverController{0};
  frc2::CommandXboxController m_operatorController{1};

  ControllerAxis vx_axis{m_driverController, frc::PS5Controller::Axis::kLeftY, true};
  ControllerAxis vy_axis{m_driverController, frc::PS5Controller::Axis::kLeftX, true};
  ControllerAxis omega_axis{m_driverController, frc::PS5Controller::Axis::kRightX, true};
  ControllerAxis arm_angle_axis{ m_operatorController, frc::XboxController::Axis::kLeftY, true };
  ControllerAxis wrist_angle_axis{ m_operatorController, frc::XboxController::Axis::kRightY, true };
  ControllerAxis elevator_axis{ m_operatorController, frc::XboxController::Axis::kLeftX, true };

};

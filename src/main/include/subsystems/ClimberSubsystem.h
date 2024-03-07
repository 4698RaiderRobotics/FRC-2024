// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc2/command/SubsystemBase.h>

#include <rev/CANSparkFlex.h>
#include <frc/DigitalInput.h>

#include "Constants.h"

class ClimberSubsystem : public frc2::SubsystemBase {
 public:
  ClimberSubsystem();

  /**
   * Will be called periodically whenever the CommandScheduler runs.
   */
  void Periodic() override;

  void SetSpeed(double speed);

  void Home();

  double GetRotations();

  void Climb();

  bool AtLimit();

 private:
  // Components (e.g. motor controllers and sensors) should generally be
  // declared private and exposed only through public methods.

  rev::CANSparkFlex m_climberMotor{deviceIDs::kClimberID, rev::CANSparkFlex::MotorType::kBrushless};

  rev::SparkRelativeEncoder m_climberEncoder = m_climberMotor.GetEncoder();

  frc::DigitalInput m_limit{1};

  bool isZeroed{ false };
  bool isHoming{ false };
  bool isRaising{ false };
};

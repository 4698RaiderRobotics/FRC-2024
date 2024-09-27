// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <units/length.h>
#include <frc2/command/SubsystemBase.h>

#include <rev/CANSparkFlex.h>
#include <frc/DigitalInput.h>


class ClimberSubsystem : public frc2::SubsystemBase {
 public:
  ClimberSubsystem();

  /**
   * Will be called periodically whenever the CommandScheduler runs.
   */
  void Periodic() override;

  void SetSpeed(double speed);

  void Home();

  units::inch_t GetHeight();

  void Climb();

  bool AtLimit();

 private:
  // Components (e.g. motor controllers and sensors) should generally be
  // declared private and exposed only through public methods.

  rev::CANSparkFlex m_climberMotor;

  rev::SparkRelativeEncoder m_climberEncoder = m_climberMotor.GetEncoder();

  frc::DigitalInput m_limit{1};

  bool isZeroed{ false };
  bool isHoming{ false };
  bool isRaising{ false };


  const double kHomingSpeed = 0.1;
  const units::inch_t kSpoolDiameter = 1.0_in;
  const double kGearRatio = 25;
};

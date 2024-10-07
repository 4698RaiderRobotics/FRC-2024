// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <units/length.h>
#include <frc/DigitalInput.h>
#include <frc/controller/PIDController.h>
#include <frc2/command/SubsystemBase.h>

#include <rev/CANSparkFlex.h>


class ClimberSubsystem : public frc2::SubsystemBase {
 public:
  ClimberSubsystem();

  void Periodic() override;

  units::inch_t GetHeight();
  void SetGoal(  units::inch_t goal );

  bool AtLimit();
  bool AtGoal();

  /**
   *  Create a command to move the climber hooks to a height.
   */
  [[nodiscard]]
  frc2::CommandPtr MoveHooks( units::inch_t h );

 private:
  void Home();
  void TrackGoal();

  rev::CANSparkFlex m_climberMotor;
  rev::SparkRelativeEncoder m_climberEncoder = m_climberMotor.GetEncoder();
  frc::PIDController m_pid{ 0.05, 0.0, 0.0 };
  frc::DigitalInput m_limit{1};

  bool isZeroed{ false };
  bool isHoming{ false };

  units::inch_t m_goal;

  const double kHomingSpeed = 0.1;
  const units::inch_t kSpoolDiameter = 1.0_in;
  const double kGearRatio = 25;
};

// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc/DigitalInput.h>
#include <frc2/command/SubsystemBase.h>

#include <frc/controller/SimpleMotorFeedforward.h>
#include <frc/controller/PIDController.h>
#include <frc/trajectory/TrapezoidProfile.h>

#include <rev/CANSparkFlex.h>

class ClimberSubsystem : public frc2::SubsystemBase {
 public:
  ClimberSubsystem();

  /**
   * Will be called periodically whenever the CommandScheduler runs.
   */
  void Periodic() override;

  void GoToHeight(units::meter_t heightGoal);

  void NudgeHeight(units::meter_t deltaHeight);

  units::inch_t GetHeight();

  bool IsAtGoal();

  bool AtLimit();

 private:
  void Home();
  void SetOpenloopSpeed(double percent);

  using Feedforward = frc::SimpleMotorFeedforward<units::meter>;
  using TrapProfile = frc::TrapezoidProfile<units::meter>;

  rev::CANSparkFlex m_Motor;
  rev::SparkRelativeEncoder m_Encoder = m_Motor.GetEncoder();

  frc::PIDController m_PID;
  Feedforward m_Feedforward;
  
  TrapProfile m_Profile;
  TrapProfile::State m_Goal;
  TrapProfile::State m_Setpoint;

  units::meter_t m_Position;

  frc::DigitalInput m_limit{1};

  bool isZeroed{ false };
  bool isHoming{ false };
  bool homingCanceled{ false };


  const double kHomingSpeed = 0.15;
  const units::inch_t kSpoolDiameter = 1.0_in;
  const double kGearRatio = 25;
};

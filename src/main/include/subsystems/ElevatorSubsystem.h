// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc2/command/SubsystemBase.h>

#include <frc/controller/ElevatorFeedforward.h>
#include <frc/controller/PIDController.h>
#include <frc/trajectory/TrapezoidProfile.h>

#include <rev/CANSparkFlex.h>

class ElevatorSubsystem : public frc2::SubsystemBase {
 public:
  ElevatorSubsystem();

  /**
   * Will be called periodically whenever the CommandScheduler runs.
   */
  void Periodic() override;

  void SetGoal( units::meter_t elevatorHeightGoal );

  void NudgeHeight(units::meter_t deltaHeight);

  units::meter_t GetHeight();

  bool AtGoal();

/**
   *  Create a command to move the arm and wrist to the specified angles.
   */
  [[nodiscard]]
  frc2::CommandPtr ChangeHeight( units::meter_t elevatorHeightGoal );

 private:
  // Components (e.g. motor controllers and sensors) should generally be
  // declared private and exposed only through public methods.

  rev::CANSparkFlex m_elevatorMotor;
  rev::SparkRelativeEncoder m_elevatorEncoder = m_elevatorMotor.GetEncoder();

  frc::PIDController m_elevatorPID;
  frc::ElevatorFeedforward m_elevatorFeedforward;
  
  frc::TrapezoidProfile<units::meters> m_elevatorProfile;
  frc::TrapezoidProfile<units::meters>::State m_elevatorGoal;
  frc::TrapezoidProfile<units::meters>::State m_elevatorSetpoint;

  units::meter_t m_elevatorPosition;

  const units::inch_t kSprocketlDiameter = 1.1235_in;  // ??? Where did this come from
  const double kGearRatio = 15;
};

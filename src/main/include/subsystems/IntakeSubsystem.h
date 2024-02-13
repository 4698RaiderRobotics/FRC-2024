// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc2/command/SubsystemBase.h>

#include <rev/CANSparkMax.h>

#include "Constants.h"

class IntakeSubsystem : public frc2::SubsystemBase {
 public:
  IntakeSubsystem();

  /**
   * Will be called periodically whenever the CommandScheduler runs.
   */
  void Periodic() override;

  // Spins the intake at a specific speed
  void SpinIntake(double speed);

  // Gets the value of the beam break
  bool GetBeamBreak();

 private:
  // Components (e.g. motor controllers and sensors) should generally be
  // declared private and exposed only through public methods.

  // Beam Break Sensor

  rev::CANSparkMax m_intakeMotor{deviceIDs::kIntakeID, rev::CANSparkMax::MotorType::kBrushless};

 public:
  bool hasNote = false;
};

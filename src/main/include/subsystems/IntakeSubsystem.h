// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc2/command/SubsystemBase.h>

#include <rev/CANSparkMax.h>
#include <frc/DigitalInput.h>

#include "Constants.h"
#include "subsystems/LEDSubsystem.h"

class IntakeSubsystem : public frc2::SubsystemBase {
 public:
  IntakeSubsystem(LEDSubsystem* leds);

  /**
   * Will be called periodically whenever the CommandScheduler runs.
   */
  void Periodic() override;

  // Spins the intake at a specific speed
  void SpinIntake(double speed);

  // Gets the value of the beam break
  bool IsBeamBroken();

  double GetRotations();

    // We picked up a Note.
  void NotePickedUp();

    // Do we currently have a note.
  bool HasNote();

 private:
  // Components (e.g. motor controllers and sensors) should generally be
  // declared private and exposed only through public methods.

  LEDSubsystem* m_leds;

  rev::CANSparkMax m_intakeMotor{deviceIDs::kIntakeID, rev::CANSparkMax::MotorType::kBrushless};

  rev::SparkRelativeEncoder m_intakeEncoder = m_intakeMotor.GetEncoder();

  frc::DigitalInput m_beamBreak{0};   /* True when beam is NOT broken */

  double m_startPos;

    // We have a note?
  bool m_hasNote = false;
    // We are moving the note backward to center it
  bool m_reversing = false;
    // We have reversed to the beam break
  bool m_isIndexed = false;
    // We are centered up
  bool m_centered = false;
};

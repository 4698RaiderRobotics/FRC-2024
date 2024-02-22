// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include <frc/smartdashboard/SmartDashboard.h>

#include "subsystems/IntakeSubsystem.h"

IntakeSubsystem::IntakeSubsystem() = default;

// This method will be called once per scheduler run
void IntakeSubsystem::Periodic() {
    frc::SmartDashboard::PutBoolean( "Beam Broken", IsBeamBroken() );
}

void IntakeSubsystem::SpinIntake(double speed) {
    m_intakeMotor.Set(speed);
}

bool IntakeSubsystem::IsBeamBroken() {
    return !m_beamBreak.Get();
}

double IntakeSubsystem::GetRotations() {
    return m_intakeEncoder.GetPosition();
}
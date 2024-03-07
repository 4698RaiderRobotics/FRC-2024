// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include <frc/smartdashboard/SmartDashboard.h>

#include "subsystems/IntakeSubsystem.h"

IntakeSubsystem::IntakeSubsystem(LEDSubsystem* leds)
 : m_leds{leds} {

};

// This method will be called once per scheduler run
void IntakeSubsystem::Periodic() {
    frc::SmartDashboard::PutBoolean( "Beam Broken", IsBeamBroken() );

    if(hasNote) {
        m_leds->SetColor(0, 255, 0);
    } else {
        m_leds->SetColor(0, 0, 0);
    }

    if(centering && IsBeamBroken()) {
            // Start moving the note back out of the intake.
        m_intakeMotor.Set(-0.2);
        isIndexed = false;
    } else if( centering && !IsBeamBroken()) {
            // Not has been picked up but not indexed.
        hasNote = true;
        centering = false;
        m_intakeMotor.Set(0.0);
        isIndexed = true;
        m_startPos = GetRotations();
        m_intakeMotor.Set(0.2);
    } else if(isIndexed && GetRotations() - m_startPos > 1.5) {
            // Note has been backed out to the resting position.
        isIndexed = false;
        m_intakeMotor.Set(0.0);
    }
}

void IntakeSubsystem::SpinIntake(double speed) {
    if(hasNote) {
        hasNote = false;
    }
    m_intakeMotor.Set(speed);
}

bool IntakeSubsystem::IsBeamBroken() {
    return !m_beamBreak.Get();
}

double IntakeSubsystem::GetRotations() {
    return m_intakeEncoder.GetPosition();
}
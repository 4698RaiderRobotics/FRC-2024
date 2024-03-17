// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include <frc/smartdashboard/SmartDashboard.h>

#include "DataLogger.h"

#include "subsystems/IntakeSubsystem.h"

IntakeSubsystem::IntakeSubsystem(LEDSubsystem* leds)
 : m_leds{leds} {

};

// This method will be called once per scheduler run
void IntakeSubsystem::Periodic() {

    DataLogger::GetInstance().SendNT( "IntakeSubsys/BeamBroken", IsBeamBroken() );
    DataLogger::GetInstance().SendNT( "IntakeSubsys/Speed", m_intakeMotor.Get() );
    DataLogger::GetInstance().SendNT( "IntakeSubsys/HasNote", m_hasNote );

    if( !m_hasNote ) {
         // If we dont have a note, nothing to do
       return;
    }

        // We have a note.
    if( m_centered ) {
        // Nothing to do.  Note is centered already.
        return;
    }

        // Do we need to reverse the note.
    if( !m_reversing && !m_centered && !m_isIndexed) {
        m_intakeMotor.Set(-0.1);
            // We have reversed a little bit to debounce the sensor if 
            // it is at the edge of the note.
        // fmt::print("    IntakeSubsystem -- Reversing note r{}\n", GetRotations() - m_startPos);
        if( GetRotations() - m_startPos < 0.0 ) {
            m_reversing = true;
        }
    } else if( m_reversing && !IsBeamBroken()) {
            // Note is reversing and the beam has become unbroken again.
            // We now go forward until the note is centered.
        m_reversing = false;
        m_isIndexed = true;
        // fmt::print("    IntakeSubsystem -- Indexed r{}\n", GetRotations() - m_startPos);
        m_intakeMotor.Set(0.0);
        m_startPos = GetRotations();
        m_intakeMotor.Set(0.1);
    } else if(m_isIndexed && GetRotations() - m_startPos > 3.5) {
            // Note has been backed out to the resting position.
        // fmt::print("    IntakeSubsystem -- Note is in resting position r{}\n", GetRotations());
        m_centered = true;
        m_intakeMotor.Set(0.0);
    }
}

void IntakeSubsystem::SpinIntake(double speed) {
    if(m_hasNote) {
        m_hasNote = false;
        m_leds->SetColor(0, 0, 0);
    }
    // fmt::print("    IntakeSubsystem -- Set Speed to {}...\n", speed );
    m_intakeMotor.Set(speed);
}

bool IntakeSubsystem::IsBeamBroken() {
    return !m_beamBreak.Get();
}

double IntakeSubsystem::GetRotations() {
    return m_intakeEncoder.GetPosition();
}

void IntakeSubsystem::NotePickedUp() {
    m_leds->SetColor(0, 255, 0);
    m_hasNote = true;
    m_centered = false;
    m_isIndexed = false;
    m_reversing = false;
    m_intakeMotor.Set(0);
    m_startPos = GetRotations();
}

bool IntakeSubsystem::HasNote() {
    return m_hasNote;
}

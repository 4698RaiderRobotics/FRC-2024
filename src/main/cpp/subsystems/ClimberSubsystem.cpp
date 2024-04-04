// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "subsystems/ClimberSubsystem.h"

#include <frc/DriverStation.h>
#include <frc/smartdashboard/SmartDashboard.h>

ClimberSubsystem::ClimberSubsystem() {
    m_climberMotor.SetSmartCurrentLimit(30);
    m_climberMotor.EnableVoltageCompensation(12);
};

// This method will be called once per scheduler run
void ClimberSubsystem::Periodic() {

    if ( frc::DriverStation::IsDisabled() ) {
        return;
    }

    frc::SmartDashboard::PutNumber("Climber Current", m_climberMotor.GetOutputCurrent());
    frc::SmartDashboard::PutNumber("Climber Rotations", GetRotations());
    frc::SmartDashboard::PutBoolean("isZeroed", isZeroed);
    frc::SmartDashboard::PutBoolean("Limit Switch", AtLimit());

    if( !isZeroed ) {
        if( !isHoming ) {
            fmt::print("   ClimberSubsystem::Periodic() -- Starting homing routine...\n");
            isHoming = true;
            m_climberMotor.Set(-0.3);
        }
    }

    if( isHoming ) {
        Home();
    }
}

void ClimberSubsystem::SetSpeed(double speed) {
    if( isHoming ) {
        if( fabs( speed ) < 0.01 ) {
                // Ignore setting a zero speed while homing.
            return;
        } else {
                // Speed was set to non-zero value during homing (Nudge was done)
                // Stop homing routine
            isHoming = false;
            isZeroed = true;
        }
    }
    
    if( speed < 0.0 && AtLimit() ) {
        // We are trying to go down and the bottom limit is tripped!
        // Stop the motor...
        fmt::print("   ClimberSubsystem::Periodic() STOPPED MOTOR from Bottoming.\n");

        m_climberMotor.Set( 0.0 );
        return;
    }

    m_climberMotor.Set(speed);
}

void ClimberSubsystem::Home() {
    // fmt::print("   ClimberSubsystem::Home() -- isHoming({}), isZeroed({}), isRaising({})\n", isHoming, isZeroed, isRaising );
    if( !isRaising ) {
        if( AtLimit() ) {
            fmt::print("   ClimberSubsystem::Home() -- Found Home...\n");
            m_climberEncoder.SetPosition(0.0);
            isZeroed = true;
            isRaising = true;
            m_climberMotor.Set( 0.5 );
        }
    } else {
        // We are raising the hooks.
        if( m_climberEncoder.GetPosition() > 300 ) {
            m_climberMotor.Set( 0.0 );
            isHoming = false;
            isRaising = false;
        }
    }
}

double ClimberSubsystem::GetRotations() {
    return m_climberEncoder.GetPosition();
}

bool ClimberSubsystem::AtLimit() {
    return !m_limit.Get();
}


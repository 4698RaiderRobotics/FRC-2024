// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include <frc/DriverStation.h>
#include <frc/smartdashboard/SmartDashboard.h>

#include "DataLogger.h"
#include "subsystems/ClimberSubsystem.h"

#include "DeviceConstants.h"
#include "Constants.h"

ClimberSubsystem::ClimberSubsystem() :
    m_climberMotor{deviceIDs::kClimberID, rev::CANSparkFlex::MotorType::kBrushless}
{
    m_climberMotor.EnableVoltageCompensation(12);

    DataLogger::SendNT( "ClimberSubsys/isZeroed", isZeroed);
    DataLogger::SendNT( "ClimberSubsys/isHoming", isHoming);
    DataLogger::SendNT( "ClimberSubsys/isRaising", isRaising);
};

// This method will be called once per scheduler run
void ClimberSubsystem::Periodic() {

    DataLogger::SendNT( "ClimberSubsys/Climber Height (in)", GetHeight().value());
    DataLogger::SendNT( "ClimberSubsys/Limit Switch", AtLimit());

    if ( frc::DriverStation::IsDisabled() ) {
        return;
    }

    DataLogger::SendNT( "ClimberSubsys/Climber Current", m_climberMotor.GetOutputCurrent());

    if( !isZeroed ) {
        if( !isHoming ) {
            fmt::print("   ClimberSubsystem::Periodic() -- Starting homing routine...\n");
            isHoming = true;
            DataLogger::SendNT( "ClimberSubsys/isHoming", isHoming);
            m_climberMotor.Set(-kHomingSpeed);
        }
    }

    if( isHoming ) {
        Home();
        return;
    }

        // We are not homing
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
            DataLogger::SendNT( "ClimberSubsys/isHoming", isHoming);
            DataLogger::SendNT( "ClimberSubsys/isZeroed", isZeroed);
            DataLogger::Log( "ClimberSubsystem::SetSpeed() -- Homing interrupted by Nudge.\n" ); 
        }
    }
    
    if( speed < 0.0 && ( AtLimit() || GetHeight() <= physical::kClimberMinHeight ) ) {
        // We are trying to go down and the bottom limit is tripped!
        // Stop the motor...
        fmt::print("   ClimberSubsystem::SetSpeed() STOPPED MOTOR from Bottoming.\n");

        m_climberMotor.Set( 0.0 );
        return;
    } else if( speed > 0.0 && GetHeight() > physical::kClimberMaxHeight ) {
        // We are trying to go up and we are at the max height
        // Ignore the request
        fmt::print("   ClimberSubsystem::SetSpeed() Climber at MAX HEIGHT.\n");

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
            DataLogger::SendNT( "ClimberSubsys/isZeroed", isZeroed);
            DataLogger::SendNT( "ClimberSubsys/isRaising", isRaising);
            m_climberMotor.Set( 0.5 );
        }
    } else {
        // We are raising the hooks.
        if( GetHeight() >= physical::kClimberRestHeight ) {
            m_climberMotor.Set( 0.0 );
            isHoming = false;
            isRaising = false;
            DataLogger::SendNT( "ClimberSubsys/isHoming", isHoming);
            DataLogger::SendNT( "ClimberSubsys/isRaising", isRaising);
        }
    }
}

units::inch_t ClimberSubsystem::GetHeight() {
    return m_climberEncoder.GetPosition() * kSpoolDiameter * units::constants::detail::PI_VAL / kGearRatio;
}

bool ClimberSubsystem::AtLimit() {
    return !m_limit.Get();
}


// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include <frc/DriverStation.h>
#include <frc2/command/CommandPtr.h>
#include <frc2/command/Commands.h>

#include "DataLogger.h"
#include "subsystems/ClimberSubsystem.h"

#include "DeviceConstants.h"
#include "Constants.h"

ClimberSubsystem::ClimberSubsystem() :
    m_climberMotor{deviceIDs::kClimberID, rev::CANSparkFlex::MotorType::kBrushless}
{
    m_climberMotor.EnableVoltageCompensation(12);

    m_goal = -1_in; /* value while homing */

    DataLogger::SendNT( "ClimberSubsys/isZeroed", isZeroed);
    DataLogger::SendNT( "ClimberSubsys/isHoming", isHoming);
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

        // We are not homing. Track the goal.
    TrackGoal();
}

void ClimberSubsystem::SetGoal( units::inch_t goal ) {
    m_goal = goal;
    if( m_goal > physical::kClimberMaxHeight ) {
        m_goal = physical::kClimberMaxHeight;
    } else if( m_goal < physical::kClimberMinHeight ) {
        m_goal = physical::kClimberMinHeight;
    }
}


void ClimberSubsystem::TrackGoal() {
    double speed = m_climberMotor.Get();
    if( speed < 0.0 && AtLimit() ) {
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

    m_climberMotor.Set( m_pid.Calculate( GetHeight().value(), m_goal.value() ) );
}

void ClimberSubsystem::Home() {
    // fmt::print("   ClimberSubsystem::Home() -- isHoming({}), isZeroed({}), isRaising({})\n", isHoming, isZeroed, isRaising );
    if( m_goal >= 0.0_in ) {
            // Goal was set to non-zero value during homing (Nudge was probably done)
            // Stop homing routine.  Call the current position zero.
        isHoming = false;
        isZeroed = true;
        m_goal = 0.0_in;
        m_climberEncoder.SetPosition(0.0);
        DataLogger::SendNT( "ClimberSubsys/isHoming", isHoming);
        DataLogger::SendNT( "ClimberSubsys/isZeroed", isZeroed);
        DataLogger::Log( "ClimberSubsystem::SetSpeed() -- Homing interrupted by Nudge.\n" ); 
        return;
    }

    if( AtLimit() ) {
        fmt::print("   ClimberSubsystem::Home() -- Found Home...\n");
        m_climberMotor.Set( 0.0 );
        isHoming = false;
        isZeroed = true;
        m_climberEncoder.SetPosition(0.0);
        m_goal = physical::kClimberRestHeight;
        DataLogger::SendNT( "ClimberSubsys/isHoming", isHoming);
        DataLogger::SendNT( "ClimberSubsys/isZeroed", isZeroed);
    }
}

units::inch_t ClimberSubsystem::GetHeight() {
    return m_climberEncoder.GetPosition() * kSpoolDiameter * units::constants::detail::PI_VAL / kGearRatio;
}

bool ClimberSubsystem::AtLimit() {
    return !m_limit.Get();
}

bool ClimberSubsystem::AtGoal() {

  if( m_climberMotor.Get() < 0.0 && AtLimit() ) {
    return true;
  }
  
  units::inch_t height_error = units::math::abs( GetHeight() - m_goal );

      // Within 0.25 inch is at target
  return height_error < 0.25_in;
}

frc2::CommandPtr ClimberSubsystem::MoveHooks( units::inch_t h ) {
    return frc2::cmd::Run( [this, h] { 
            SetGoal( h ); 
        } )
        .Until( [this] { return AtGoal(); } )
        .WithName( "MoveHooks" );
}


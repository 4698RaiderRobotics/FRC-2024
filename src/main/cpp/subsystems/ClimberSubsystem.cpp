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
    m_Motor{deviceIDs::kClimberID, rev::CANSparkFlex::MotorType::kBrushless},
    m_PID{pidf::kClimberP, pidf::kClimberI, pidf::kClimberD},
    m_Feedforward{units::volt_t{pidf::kClimberS}, 
                  units::unit_t<Feedforward::kv_unit> {pidf::kClimberV}, 
                  units::unit_t<Feedforward::ka_unit> {pidf::kClimberA}},
    m_Profile{{physical::kClimberMaxSpeed, physical::kClimberMaxAcceleration}}
{
    m_Motor.EnableVoltageCompensation(12);

    m_Goal.position = -1_in; /* value while homing */

    DataLogger::SendNT( "ClimberSubsys/isZeroed", isZeroed);
    DataLogger::SendNT( "ClimberSubsys/isHoming", isHoming);
    DataLogger::SendNT( "ClimberSubsys/homingCanceled", homingCanceled);
};

// This method will be called once per scheduler run
void ClimberSubsystem::Periodic() {
    m_Position = GetHeight();
    DataLogger::SendNT( "ClimberSubsys/Height", units::inch_t(m_Position).value() );
    DataLogger::SendNT( "ClimberSubsys/Limit Switch", AtLimit());

    if (frc::DriverStation::IsDisabled()) {
        m_Setpoint.position = m_Position;
        m_Setpoint.velocity = 0_mps;

        m_Goal.position = m_Position;
        m_Goal.velocity = 0_mps;
        
        return;
    }

    units::meters_per_second_t mech_velocity = units::revolutions_per_minute_t(m_Encoder.GetVelocity())
                                               * kSpoolDiameter * units::constants::detail::PI_VAL / ( kGearRatio * 1_tr );
    DataLogger::SendNT( "ClimberSubsys/Velocity(mps)", mech_velocity.value() );
    DataLogger::SendNT( "ClimberSubsys/Climber Current", m_Motor.GetOutputCurrent());

    if( !isZeroed && !homingCanceled ) {
        if( isHoming ) {
                // we already starting homing. Look for Limit switch
            Home();
        } else  {
                // Start homing ...
            fmt::print("   ClimberSubsystem::Periodic() -- Starting homing routine...\n");
            isHoming = true;
            DataLogger::SendNT( "ClimberSubsys/isHoming", isHoming);
            SetOpenloopSpeed(-kHomingSpeed);
        }
        return;
    }

    DataLogger::SendNT( "ClimberSubsys/Goal Height", units::inch_t(m_Goal.position).value() );
    DataLogger::SendNT( "ClimberSubsys/AtGoal", AtGoal() );
    DataLogger::SendNT( "ClimberSubsys/Spt Position", units::inch_t(m_Setpoint.position).value() );
    DataLogger::SendNT( "ClimberSubsys/Spt Velocity(mps)", m_Setpoint.velocity.value() );

        // We are not homing.  Track the goal height....
    m_Setpoint = m_Profile.Calculate(physical::kDt, m_Setpoint, m_Goal);

    double PIDOutput = m_PID.Calculate(m_Position.value(), m_Setpoint.position.value());
    double FFOutput = m_Feedforward.Calculate(m_Setpoint.velocity).value();

    SetOpenloopSpeed(PIDOutput + FFOutput / 12);
}

void ClimberSubsystem::SetGoal( units::inch_t goal ) {
    m_Goal.position = goal;
    if( m_Goal.position > physical::kClimberMaxHeight ) {
        m_Goal.position = physical::kClimberMaxHeight;
    } else if( m_Goal.position < physical::kClimberMinHeight ) {
        m_Goal.position = physical::kClimberMinHeight;
    }
}

void ClimberSubsystem::SetOpenloopSpeed(double percent) {
    if( percent < 0.0 && AtLimit() ) {
        // We are trying to go down and the bottom limit is tripped!
        // Stop the motor...
        fmt::print("   ClimberSubsystem::SetOpenloopSpeed() STOPPED MOTOR from Bottoming.\n");

        m_Motor.Set( 0.0 );
        return;
    } else if( percent > 0.0 && GetHeight() > physical::kClimberMaxHeight ) {
        // We are trying to go up and we are at the max height
        // Ignore the request
        fmt::print("   ClimberSubsystem::SetOpenloopSpeed() Climber at MAX HEIGHT.\n");

        m_Motor.Set( 0.0 );
        return;
    }

    m_Motor.Set(percent);
}

void ClimberSubsystem::Home() {
    if( m_Goal.position >= 0.0_in ) {
            // Goal was set to non-zero value during homing (Nudge was probably done)
            // Stop homing routine.  Call the current position zero.
        isHoming = false;
        isZeroed = true;
        m_Goal.position = 0.0_in;
        m_Encoder.SetPosition(0.0);
        DataLogger::SendNT( "ClimberSubsys/isHoming", isHoming);
        DataLogger::SendNT( "ClimberSubsys/isZeroed", isZeroed);
        DataLogger::Log( "ClimberSubsystem::SetSpeed() -- Homing interrupted by Nudge.\n" ); 
        return;
    }

            // We are still going down looking for the Limit Switch
    if( AtLimit() ) {
            // We hit the limit switch!  Stop homing and set the position to zero and set the setpoint to zero.
        fmt::print("   ClimberSubsystem::Home() -- Found Home...\n");
        m_Encoder.SetPosition(0.0);
        m_Setpoint.position = 0.0_in;
        isZeroed = true;
        isHoming = false;
        DataLogger::SendNT( "ClimberSubsys/isZeroed", isZeroed);
        DataLogger::SendNT( "ClimberSubsys/isHoming", isHoming);
        SetGoal( physical::kClimberRestHeight );
    }
}

units::inch_t ClimberSubsystem::GetHeight() {
    return m_Encoder.GetPosition() * kSpoolDiameter * units::constants::detail::PI_VAL / kGearRatio;
}

bool ClimberSubsystem::AtLimit() {
    return !m_limit.Get();
}

bool ClimberSubsystem::AtGoal() {

  if( AtLimit() ) {
    return true;
  }
  
  units::inch_t height_error = units::math::abs( GetHeight() - m_Goal.position );

      // Within 0.5 inch is at target
  return height_error < 0.5_in;
}

frc2::CommandPtr ClimberSubsystem::MoveHooks( units::inch_t h ) {
    return frc2::cmd::Sequence(
        RunOnce( [this, h] { SetGoal( h ); } ),
        frc2::cmd::WaitUntil( [this] { return AtGoal(); } ).WithTimeout( 3_s )
    ).WithName( "MoveHooks" );
}


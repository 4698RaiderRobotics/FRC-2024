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
    m_Motor{deviceIDs::kClimberID, rev::CANSparkFlex::MotorType::kBrushless},
    m_PID{pidf::kClimberP, pidf::kClimberI, pidf::kClimberD},
    m_Feedforward{units::volt_t{pidf::kClimberS}, units::volt_t{pidf::kClimberG}, 
                          units::unit_t<frc::ElevatorFeedforward::kv_unit> {pidf::kClimberV}, 
                          units::unit_t<frc::ElevatorFeedforward::ka_unit> {pidf::kClimberA}},
    m_Profile{{physical::kClimberMaxSpeed, physical::kClimberMaxAcceleration}}
{
    m_Motor.SetSmartCurrentLimit(30);
    m_Motor.EnableVoltageCompensation(12);

    DataLogger::GetInstance().SendNT( "ClimberSubsys/isZeroed", isZeroed);
    DataLogger::GetInstance().SendNT( "ClimberSubsys/isHoming", isHoming);
    DataLogger::GetInstance().SendNT( "ClimberSubsys/homingCanceled", homingCanceled);
};

// This method will be called once per scheduler run
void ClimberSubsystem::Periodic() {
    m_Position = GetHeight();
    DataLogger::GetInstance().SendNT( "ClimberSubsys/Height", units::inch_t(m_Position).value() );
    DataLogger::GetInstance().SendNT( "ClimberSubsys/Limit Switch", AtLimit());

    if (frc::DriverStation::IsDisabled()) {
        m_Setpoint.position = m_Position;
        m_Setpoint.velocity = 0_mps;

        m_Goal.position = m_Position;
        m_Goal.velocity = 0_mps;
        
        return;
    }

    units::meters_per_second_t mech_velocity = units::revolutions_per_minute_t(m_Encoder.GetVelocity())
                                               * kSpoolDiameter * units::constants::detail::PI_VAL / ( kGearRatio * 1_tr );
    DataLogger::GetInstance().SendNT( "ClimberSubsys/Velocity(m/s)", mech_velocity.value() );
    DataLogger::GetInstance().SendNT( "ClimberSubsys/Climber Current", m_Motor.GetOutputCurrent());

    if( !isZeroed && !homingCanceled ) {
        if( isHoming ) {
                // we already starting homing. Look for Limit switch
            Home();
        } else  {
                // Start homing ...
            fmt::print("   ClimberSubsystem::Periodic() -- Starting homing routine...\n");
            isHoming = true;
            DataLogger::GetInstance().SendNT( "ClimberSubsys/isHoming", isHoming);
            SetOpenloopSpeed(-kHomingSpeed);
        }
        return;
    }

    DataLogger::GetInstance().SendNT( "ClimberSubsys/Goal Height", units::inch_t(m_Goal.position).value() );
    DataLogger::GetInstance().SendNT( "ClimberSubsys/IsAtGoal", IsAtGoal() );
    DataLogger::GetInstance().SendNT( "ClimberSubsys/Goal Velocity(m/s)", m_Setpoint.velocity.value() );

        // We are not homing.  Track the goal height....
    m_Setpoint = m_Profile.Calculate(physical::kDt, m_Setpoint, m_Goal);

    double PIDOutput = m_PID.Calculate(m_Position.value(), m_Setpoint.position.value());
    double FFOutput = m_Feedforward.Calculate(m_Setpoint.velocity).value();

    SetOpenloopSpeed(PIDOutput + FFOutput / 12);
}

void ClimberSubsystem::GoToHeight(units::meter_t heightGoal) {
    if( isHoming ) { 
            // We are homing and got a command to move to a certain height!
            // Probably a Nudge was done during homing.
            // The best we can do is call the current position zero.
        isHoming = false;
        isZeroed = false;
        homingCanceled = true;

        m_Encoder.SetPosition(0.0);
        m_Setpoint.position = 0.0_in;

        DataLogger::GetInstance().SendNT( "ClimberSubsys/isHoming", isHoming);
        DataLogger::GetInstance().SendNT( "ClimberSubsys/isZeroed", isZeroed);
        DataLogger::GetInstance().SendNT( "ClimberSubsys/homingCanceled", homingCanceled);
        DataLogger::GetInstance().Log( "ClimberSubsystem::GoToHeight() -- Homing interrupted by Nudge.\n" ); 
    }
    
    if(heightGoal > physical::kClimberMaxHeight) {heightGoal = physical::kClimberMaxHeight;}
    if(heightGoal < physical::kClimberMinHeight) {heightGoal = physical::kClimberMinHeight;}

    m_Goal.position = heightGoal;
}

void ClimberSubsystem::NudgeHeight(units::meter_t deltaHeight) {
    GoToHeight( m_Goal.position + deltaHeight );
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
    // fmt::print("   ClimberSubsystem::Home() -- isHoming({}), isZeroed({}), homingCanceled({})\n", isHoming, isZeroed, homingCanceled );

            // We are still going down looking for the Limit Switch
    if( AtLimit() ) {
            // We hit the limit switch!  Stop homing and set the position to zero and set the setpoint to zero.
        fmt::print("   ClimberSubsystem::Home() -- Found Home...\n");
        m_Encoder.SetPosition(0.0);
        m_Setpoint.position = 0.0_in;
        isZeroed = true;
        isHoming = false;
        DataLogger::GetInstance().SendNT( "ClimberSubsys/isZeroed", isZeroed);
        DataLogger::GetInstance().SendNT( "ClimberSubsys/isHoming", isHoming);
        GoToHeight( physical::kClimberRestHeight );
    }
}

units::inch_t ClimberSubsystem::GetHeight() {
    return m_Encoder.GetPosition() * kSpoolDiameter * units::constants::detail::PI_VAL / kGearRatio;
}

bool ClimberSubsystem::IsAtGoal() {
    return units::math::abs( m_Goal.position - m_Position ) < 1_in;
}

bool ClimberSubsystem::AtLimit() {
    return !m_limit.Get();
}


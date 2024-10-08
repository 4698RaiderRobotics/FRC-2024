// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include <frc/DriverStation.h>
#include <frc2/command/Commands.h>

#include "DataLogger.h"
#include "subsystems/ElevatorSubsystem.h"

#include "DeviceConstants.h"
#include "Constants.h"


ElevatorSubsystem::ElevatorSubsystem() :
    m_elevatorMotor{deviceIDs::kElevatorID, rev::CANSparkFlex::MotorType::kBrushless},
    m_elevatorPID{pidf::kElevatorP, pidf::kElevatorI, pidf::kElevatorD},
    m_elevatorFeedforward{units::volt_t{pidf::kElevatorS}, units::volt_t{pidf::kElevatorG}, 
                          units::unit_t<frc::ElevatorFeedforward::kv_unit> {pidf::kElevatorV}, 
                          units::unit_t<frc::ElevatorFeedforward::ka_unit> {pidf::kElevatorA}},
    m_elevatorProfile{{physical::kElevatorMaxSpeed, physical::kElevatorMaxAcceleration}}
{
    m_elevatorMotor.EnableVoltageCompensation(12);
    m_elevatorMotor.SetInverted(true);

    m_elevatorEncoder.SetPosition(0.0);
};

// This method will be called once per scheduler run
void ElevatorSubsystem::Periodic() {

    m_elevatorPosition = m_elevatorEncoder.GetPosition() / 15.0 * units::constants::detail::PI_VAL * 1.1235 * 2.0 * 0.0254_m;
    units::meters_per_second_t linearVel = m_elevatorEncoder.GetVelocity() * 1_rpm / 15.0_tr * units::constants::detail::PI_VAL * 1.1235 * 2.0 * 0.0254_m;
    DataLogger::SendNT( "ElevatorSubsys/Height", units::inch_t(m_elevatorPosition).value() );
    DataLogger::SendNT( "ElevatorSubsys/Velocity(mps)", linearVel.value() );

    if (frc::DriverStation::IsDisabled()) {
        m_elevatorSetpoint.position = m_elevatorPosition;
        m_elevatorSetpoint.velocity = 0_mps;

        m_elevatorGoal.position = m_elevatorPosition;
        m_elevatorGoal.velocity = 0_mps;
        
        return;
    }

    DataLogger::SendNT( "ElevatorSubsys/Goal Height", units::inch_t(m_elevatorGoal.position).value() );
    DataLogger::SendNT( "ElevatorSubsys/Spt Velocity(mps)", m_elevatorSetpoint.velocity.value() );
    DataLogger::SendNT( "ElevatorSubsys/IsAtGoal", AtGoal() );
    DataLogger::SendNT( "ElevatorSubsys/Current", m_elevatorMotor.GetOutputCurrent());

    m_elevatorSetpoint = m_elevatorProfile.Calculate(physical::kDt, m_elevatorSetpoint, m_elevatorGoal);

    double elevatorOutput = m_elevatorPID.Calculate(m_elevatorPosition.value(), m_elevatorSetpoint.position.value());
    double elevatorFFOutput = m_elevatorFeedforward.Calculate(m_elevatorSetpoint.velocity).value();

    // frc::SmartDashboard::PutNumber("Elevator Height", m_elevatorPosition.value());

    m_elevatorMotor.Set(elevatorOutput + elevatorFFOutput / 12);
}

void ElevatorSubsystem::SetGoal(units::meter_t elevatorHeightGoal) {
    if(elevatorHeightGoal > physical::kElevatorMaxHeight) {elevatorHeightGoal = physical::kElevatorMaxHeight;}
    if(elevatorHeightGoal < physical::kElevatorMinHeight) {elevatorHeightGoal = physical::kElevatorMinHeight;}

    m_elevatorGoal.position = elevatorHeightGoal;
}

void ElevatorSubsystem::NudgeHeight(units::meter_t deltaHeight) {
    SetGoal( m_elevatorGoal.position + deltaHeight );
}

units::meter_t ElevatorSubsystem::GetHeight() {
    return m_elevatorPosition;
}

bool ElevatorSubsystem::AtGoal() {
    return units::math::abs( m_elevatorGoal.position - m_elevatorPosition ) < 1_in;
}

frc2::CommandPtr ElevatorSubsystem::ChangeHeight( units::meter_t goal ) {
    return frc2::cmd::Sequence(
        Run( [this, goal] { SetGoal( goal ); }),
        frc2::cmd::WaitUntil( [this] { return AtGoal(); } )
    )
    .WithTimeout( 3_s )
    .WithName( "ChangeHeight" );
}

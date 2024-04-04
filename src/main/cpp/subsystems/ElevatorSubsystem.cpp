// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include <frc/DriverStation.h>
#include <frc/smartdashboard/SmartDashboard.h>

#include "DataLogger.h"
#include "subsystems/ElevatorSubsystem.h"


ElevatorSubsystem::ElevatorSubsystem() {
    m_elevatorEncoder.SetPosition(0.0);
};

// This method will be called once per scheduler run
void ElevatorSubsystem::Periodic() {

    m_elevatorPosition = m_elevatorEncoder.GetPosition() / 15.0 * units::constants::detail::PI_VAL * 1.1235 * 2.0 * 0.0254_m;
    DataLogger::GetInstance().SendNT( "ElevatorSubsys/Height", m_elevatorPosition.value() );

    if (frc::DriverStation::IsDisabled()) {
        m_elevatorSetpoint.position = m_elevatorPosition;
        m_elevatorSetpoint.velocity = 0_mps;

        m_elevatorGoal.position = m_elevatorPosition;
        m_elevatorGoal.velocity = 0_mps;
        
        return;
    }

    DataLogger::GetInstance().SendNT( "ElevatorSubsys/Goal Height", m_elevatorGoal.position.value() );
    DataLogger::GetInstance().SendNT( "ElevatorSubsys/IsAtGoal", IsAtGoal() );

    m_elevatorSetpoint = m_elevatorProfile.Calculate(physical::kDt, m_elevatorSetpoint, m_elevatorGoal);

    double elevatorOutput = m_elevatorPID.Calculate(m_elevatorPosition.value(), m_elevatorSetpoint.position.value());
    double elevatorFFOutput = m_elevatorFeedforward.Calculate(m_elevatorSetpoint.velocity).value();

    frc::SmartDashboard::PutNumber("Elevator Height", m_elevatorPosition.value());

    m_elevatorMotor.Set(elevatorOutput + elevatorFFOutput / 12);
}

void ElevatorSubsystem::GoToHeight(units::meter_t elevatorHeightGoal) {
    if(elevatorHeightGoal > physical::kElevatorMaxHeight) {elevatorHeightGoal = physical::kElevatorMaxHeight;}
    if(elevatorHeightGoal < physical::kElevatorMinHeight) {elevatorHeightGoal = physical::kElevatorMinHeight;}

    m_elevatorGoal.position = elevatorHeightGoal;
}

void ElevatorSubsystem::NudgeHeight(units::meter_t deltaHeight) {
    m_elevatorGoal.position += deltaHeight;

    if(m_elevatorGoal.position > physical::kElevatorMaxHeight) {m_elevatorGoal.position = physical::kElevatorMaxHeight;}
    if(m_elevatorGoal.position < physical::kElevatorMinHeight) {m_elevatorGoal.position = physical::kElevatorMinHeight;}
}

units::meter_t ElevatorSubsystem::GetHeight() {
    return m_elevatorPosition;
}

bool ElevatorSubsystem::IsAtGoal() {
    return units::math::abs( m_elevatorGoal.position - m_elevatorPosition ) < 1_in;
}
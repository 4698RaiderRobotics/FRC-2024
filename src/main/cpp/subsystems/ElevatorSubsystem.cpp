// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include <frc/DriverStation.h>
#include <frc/smartdashboard/SmartDashboard.h>

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
    m_elevatorMotor.SetSmartCurrentLimit(30);
    m_elevatorMotor.EnableVoltageCompensation(12);
    m_elevatorEncoder.SetPosition(0.0);
};

// This method will be called once per scheduler run
void ElevatorSubsystem::Periodic() {

        // Elevator distance is twice (2x) the chain distance because it is two-stage.
    m_elevatorPosition = m_elevatorEncoder.GetPosition() * units::constants::detail::PI_VAL * kSprocketlDiameter * 2.0 / kGearRatio;
    DataLogger::GetInstance().SendNT( "ElevatorSubsys/Height", units::inch_t(m_elevatorPosition).value() );

    if (frc::DriverStation::IsDisabled()) {
        m_elevatorSetpoint.position = m_elevatorPosition;
        m_elevatorSetpoint.velocity = 0_mps;

        m_elevatorGoal.position = m_elevatorPosition;
        m_elevatorGoal.velocity = 0_mps;
        
        return;
    }

    units::meters_per_second_t mech_velocity = units::revolutions_per_minute_t(m_elevatorEncoder.GetVelocity())
                                               * kSprocketlDiameter * 2.0 * units::constants::detail::PI_VAL / ( kGearRatio * 1_tr );
    DataLogger::GetInstance().SendNT( "ElevatorSubsys/Velocity(m/s)", mech_velocity.value() );
    DataLogger::GetInstance().SendNT( "ElevatorSubsys/Goal Height", units::inch_t(m_elevatorGoal.position).value() );
    DataLogger::GetInstance().SendNT( "ElevatorSubsys/IsAtGoal", IsAtGoal() );

    m_elevatorSetpoint = m_elevatorProfile.Calculate(physical::kDt, m_elevatorSetpoint, m_elevatorGoal);

    double elevatorOutput = m_elevatorPID.Calculate(m_elevatorPosition.value(), m_elevatorSetpoint.position.value());
    double elevatorFFOutput = m_elevatorFeedforward.Calculate(m_elevatorSetpoint.velocity).value();

    // frc::SmartDashboard::PutNumber("Elevator Height", m_elevatorPosition.value());

    m_elevatorMotor.Set(elevatorOutput + elevatorFFOutput / 12);
}

void ElevatorSubsystem::GoToHeight(units::meter_t elevatorHeightGoal) {
    if(elevatorHeightGoal > physical::kElevatorMaxHeight) {elevatorHeightGoal = physical::kElevatorMaxHeight;}
    if(elevatorHeightGoal < physical::kElevatorMinHeight) {elevatorHeightGoal = physical::kElevatorMinHeight;}

    m_elevatorGoal.position = elevatorHeightGoal;
}

void ElevatorSubsystem::NudgeHeight(units::meter_t deltaHeight) {
    GoToHeight( m_elevatorGoal.position + deltaHeight );
}

units::meter_t ElevatorSubsystem::GetHeight() {
    return m_elevatorPosition;
}

bool ElevatorSubsystem::IsAtGoal() {
    return units::math::abs( m_elevatorGoal.position - m_elevatorPosition ) < 1_in;
}
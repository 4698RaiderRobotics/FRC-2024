// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "subsystems/ArmSubsystem.h"

#include <frc/DriverStation.h>

ArmSubsystem::ArmSubsystem() = default;

// This method will be called once per scheduler run
void ArmSubsystem::Periodic() {
    m_wristPosition = m_wristEncoder.GetPosition();
    m_armPosition = m_armEncoder.GetPosition().GetValueAsDouble() * 1_deg - m_wristPosition;

    if ( frc::DriverStation::IsDisabled() ) {
        m_armSetpoint.position = m_armPosition;
        m_armSetpoint.velocity = 0_deg_per_s;
        m_armAngleGoal = m_armPosition;

        m_wristSetpoint.position = m_wristPosition;
        m_wristSetpoint.velocity = 0_deg_per_s;
        m_wristAngleGoal = m_wristPosition;

        return;
    }

    m_wristGoal = {m_wristAngleGoal, 0_deg_per_s};
    m_armGoal = {m_armAngleGoal, 0_deg_per_s};
    
    m_wristSetpoint = m_wristProfile.Calculate(physical::kDt, m_wristSetpoint, m_wristGoal);
    m_armSetpoint = m_armProfile.Calculate(physical::kDt, m_armSetpoint, m_armGoal);


    units::degree_t alpha = 90_deg + m_wristPosition + m_armPosition;

    // Still need alpha calculation
    double wristOutput = m_wristPID.Calculate(m_wristPosition.value(), m_wristSetpoint.position.value());
    double wristFeedforwardOut = m_wristFeedforward.Calculate(alpha, m_wristSetpoint.velocity).value();

    

    double armOutput = m_armPID.Calculate(m_armPosition.value(), m_armSetpoint.position.value());
    double armFeedforwardOut = m_armFeedforward.Calculate(m_armSetpoint.position, m_armSetpoint.velocity).value();
}

void ArmSubsystem::GoToAngle(units::degree_t armAngleGoal, units::degree_t wristAngleGoal) {
    m_armAngleGoal = armAngleGoal;
    m_wristAngleGoal = wristAngleGoal;

    if(m_armAngleGoal > physical::kArmMaxAngle) m_armAngleGoal = physical::kArmMaxAngle;
    if(m_armAngleGoal < physical::kArmMinAngle) m_armAngleGoal = physical::kArmMinAngle;

    if (m_wristAngleGoal > physical::kWristMaxAngle) m_wristAngleGoal = physical::kWristMaxAngle;
    if (m_wristAngleGoal < physical::kWristMinAngle) m_wristAngleGoal = physical::kWristMinAngle;
}
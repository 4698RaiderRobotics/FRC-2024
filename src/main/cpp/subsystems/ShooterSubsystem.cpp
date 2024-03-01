// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "subsystems/ShooterSubsystem.h"

#include <frc/DriverStation.h>

ShooterSubsystem::ShooterSubsystem() {
    m_topShooterMotor.SetInverted(true);
    m_bottomShooterMotor.SetInverted(true);
    m_bottomShooterMotor.Follow(m_topShooterMotor);

    ctre::phoenix6::configs::CANcoderConfiguration absoluteEncoderConfigs{};
    absoluteEncoderConfigs.MagnetSensor.MagnetOffset = physical::kShooterAbsoluteOffset;
    m_shooterAngleEncoder.GetConfigurator().Apply(absoluteEncoderConfigs, 50_ms);

    m_angleShooterMotor.EnableVoltageCompensation(12);
    m_angleShooterMotor.SetInverted(true);

    m_speedPID.SetP(pidf::kSpeedP);
    m_speedPID.SetI(pidf::kSpeedI);
    m_speedPID.SetD(pidf::kSpeedD);
    m_speedPID.SetFF(pidf::kSpeedFF);

};

// This method will be called once per scheduler run
void ShooterSubsystem::Periodic() {
    m_shooterPosition = m_shooterAngleEncoder.GetPosition().GetValueAsDouble() * 360_deg;

    if ( frc::DriverStation::IsDisabled() ) {
        m_shooterSetpoint.position = m_shooterPosition;
        m_shooterSetpoint.velocity = 0_deg_per_s;
        m_shooterAngleGoal = m_shooterPosition;

        return;
    }

    m_shooterGoal = {m_shooterAngleGoal, 0_deg_per_s};

    m_shooterSetpoint = m_shooterProfile.Calculate(physical::kDt, m_shooterSetpoint, m_shooterGoal);

    double shooterOutput = m_shooterPID.Calculate(m_shooterPosition.value(), m_shooterSetpoint.position.value());
    double shooterFFOutput = m_shooterFeedforward.Calculate(m_shooterSetpoint.position, m_shooterSetpoint.velocity).value();

    m_angleShooterMotor.Set(shooterOutput + shooterFFOutput / 12);

    m_speedPID.SetReference(m_speed.value(), rev::CANSparkFlex::ControlType::kVelocity);
}

void ShooterSubsystem::GoToAngle(units::degree_t shooterAngleGoal) {
    m_shooterAngleGoal = shooterAngleGoal;

    if(m_shooterAngleGoal > physical::kShooterMaxAngle) {m_shooterAngleGoal = physical::kShooterMaxAngle;}
    if(m_shooterAngleGoal < physical::kShooterMinAngle) {m_shooterAngleGoal = physical::kShooterMinAngle;}
}

void ShooterSubsystem::Spin(units::revolutions_per_minute_t speed) {
    m_speed = speed;
    
}

bool ShooterSubsystem::AtSpeed() {
    return m_topEncoder.GetVelocity() >= m_speed.value();
}
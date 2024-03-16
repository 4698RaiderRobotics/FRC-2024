// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include <frc/DriverStation.h>
#include <frc/smartdashboard/SmartDashboard.h>

#include "DataLogger.h"
#include "subsystems/ShooterSubsystem.h"

ShooterSubsystem::ShooterSubsystem() {
    m_leftShooterMotor.SetInverted(true);
    m_leftShooterMotor.Follow(m_rightShooterMotor);

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

    DataLogger::GetInstance().SendNT( "ShooterSubsys/Angle", m_shooterPosition.value() );
    DataLogger::GetInstance().SendNT( "ShooterSubsys/Speed", m_rightEncoder.GetVelocity() );

    if ( frc::DriverStation::IsDisabled() ) {
        m_shooterSetpoint.position = m_shooterPosition;
        m_shooterSetpoint.velocity = 0_deg_per_s;
        m_shooterGoal.position = m_shooterPosition;
        m_shooterGoal.velocity = 0_deg_per_s;

        return;
    }

    m_shooterSetpoint = m_shooterProfile.Calculate(physical::kDt, m_shooterSetpoint, m_shooterGoal);

    double shooterOutput = m_shooterPID.Calculate(m_shooterPosition.value(), m_shooterSetpoint.position.value());
    double shooterFFOutput = m_shooterFeedforward.Calculate(m_shooterSetpoint.position, m_shooterSetpoint.velocity).value();

    m_angleShooterMotor.Set(shooterOutput + shooterFFOutput / 12);

    m_speedPID.SetReference(m_speed.value(), rev::CANSparkFlex::ControlType::kVelocity);
}

void ShooterSubsystem::GoToAngle(units::degree_t shooterAngleGoal) {
    m_shooterGoal.position = shooterAngleGoal;

    if(m_shooterGoal.position > physical::kShooterMaxAngle) {m_shooterGoal.position = physical::kShooterMaxAngle;}
    if(m_shooterGoal.position < physical::kShooterMinAngle) {m_shooterGoal.position = physical::kShooterMinAngle;}
}

void ShooterSubsystem::Spin(units::revolutions_per_minute_t speed) {
    m_speed = speed;
    
}

bool ShooterSubsystem::IsAtSpeed() {
    return m_rightEncoder.GetVelocity() >= m_speed.value() - 200;
}

units::degree_t ShooterSubsystem::GetAngle() {
    return m_shooterPosition;
}

void ShooterSubsystem::Nudge( units::degree_t deltaAngle ) {
     GoToAngle( m_shooterGoal.position + deltaAngle );
}

bool ShooterSubsystem::IsAtGoal() {
    return units::math::abs( m_shooterGoal.position - m_shooterPosition ) < 3_deg;
}
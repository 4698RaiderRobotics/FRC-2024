// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include <frc/DriverStation.h>
#include <frc/smartdashboard/SmartDashboard.h>
#include <frc/Preferences.h>

#include "DataLogger.h"
#include "subsystems/ShooterSubsystem.h"

ShooterSubsystem::ShooterSubsystem() {
    frc::Preferences::InitDouble("ShooterOffset", 0.0);

    m_leftShooterMotor.RestoreFactoryDefaults();
    m_rightShooterMotor.RestoreFactoryDefaults();

 //   m_leftShooterMotor.SetInverted( true );

    // m_leftShooterMotor.SetSmartCurrentLimit( 40 );
    // m_rightShooterMotor.SetSmartCurrentLimit( 40 );
    m_leftShooterMotor.EnableVoltageCompensation(12);
    m_rightShooterMotor.EnableVoltageCompensation(12);
    m_leftShooterMotor.SetIdleMode( rev::CANSparkMax::IdleMode::kCoast );
    m_rightShooterMotor.SetIdleMode( rev::CANSparkMax::IdleMode::kCoast );
    m_leftShooterMotor.Follow( m_rightShooterMotor, true );

    ctre::phoenix6::configs::CANcoderConfiguration absoluteEncoderConfigs{};
    absoluteEncoderConfigs.MagnetSensor.MagnetOffset = frc::Preferences::GetDouble("ShooterOffset");
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

    if ( frc::DriverStation::IsDisabled() ) {
        m_shooterSetpoint.position = m_shooterPosition;
        m_shooterSetpoint.velocity = 0_deg_per_s;
        m_shooterGoal.position = m_shooterPosition;
        m_shooterGoal.velocity = 0_deg_per_s;
        m_speed = 0_rpm;
        return;
    }

    DataLogger::GetInstance().SendNT( "ShooterSubsys/Speed", m_rightEncoder.GetVelocity() );
    DataLogger::GetInstance().SendNT( "ShooterSubsys/SpeedGoal", m_speed.value() );
    DataLogger::GetInstance().SendNT( "ShooterSubsys/left Current", m_leftShooterMotor.GetOutputCurrent() );
    DataLogger::GetInstance().SendNT( "ShooterSubsys/right Current", m_rightShooterMotor.GetOutputCurrent() );
    DataLogger::GetInstance().SendNT( "ShooterSubsys/IsAtSpeed", IsAtSpeed() );
    DataLogger::GetInstance().SendNT( "ShooterSubsys/IsAtAngle", IsAtAngle() );

    m_shooterSetpoint = m_shooterProfile.Calculate(physical::kDt, m_shooterSetpoint, m_shooterGoal);

    double shooterOutput = m_shooterPID.Calculate(m_shooterPosition.value(), m_shooterSetpoint.position.value());
    double shooterFFOutput = m_shooterFeedforward.Calculate(m_shooterSetpoint.position, m_shooterSetpoint.velocity).value();

    m_angleShooterMotor.Set(shooterOutput + shooterFFOutput / 12);

    if( m_speed < 0.01_rpm ) {
        m_rightShooterMotor.Set( 0 );
    } else {
        m_speedPID.SetReference(m_speed.value(), rev::CANSparkFlex::ControlType::kVelocity);
    }
}

void ShooterSubsystem::GoToAngle(units::degree_t shooterAngleGoal) {
    m_shooterGoal.position = shooterAngleGoal;

    if(m_shooterGoal.position > physical::kShooterMaxAngle) {m_shooterGoal.position = physical::kShooterMaxAngle;}
    if(m_shooterGoal.position < physical::kShooterMinAngle) {m_shooterGoal.position = physical::kShooterMinAngle;}
}

void ShooterSubsystem::Spin(units::revolutions_per_minute_t speed) {
    m_speed = speed;
}

units::degree_t ShooterSubsystem::GetAngle() {
    return m_shooterPosition;
}

void ShooterSubsystem::Nudge( units::degree_t deltaAngle ) {
     GoToAngle( m_shooterGoal.position + deltaAngle );
}

bool ShooterSubsystem::IsAtSpeed() {
    return m_rightEncoder.GetVelocity() >= m_speed.value() - 200;
}

bool ShooterSubsystem::IsAtAngle() {
    return units::math::abs( m_shooterGoal.position - m_shooterPosition ) < 3_deg;
}

bool ShooterSubsystem::IsAtGoal() {
    return IsAtSpeed() && IsAtAngle();
}

void ShooterSubsystem::UpdateEncoderOffset() {
    ctre::phoenix6::configs::CANcoderConfiguration shooterAbsoluteEncoderConfigs{};
    m_shooterAngleEncoder.GetConfigurator().Refresh( shooterAbsoluteEncoderConfigs );

    double offset = shooterAbsoluteEncoderConfigs.MagnetSensor.MagnetOffset - m_shooterAngleEncoder.GetAbsolutePosition().GetValueAsDouble();

    frc::Preferences::SetDouble("ShooterOffset", offset);

    shooterAbsoluteEncoderConfigs.MagnetSensor.MagnetOffset = offset;
    m_shooterAngleEncoder.GetConfigurator().Apply(shooterAbsoluteEncoderConfigs, 50_ms);
}
// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "subsystems/ShooterSubsystem.h"

ShooterSubsystem::ShooterSubsystem() {
    m_topShooterMotor.SetInverted(true);
    m_bottomShooterMotor.SetInverted(true);
    m_bottomShooterMotor.Follow(m_topShooterMotor);

    m_angleShooterMotor.EnableVoltageCompensation(12);
};

// This method will be called once per scheduler run
void ShooterSubsystem::Periodic() {
    m_shooterPosition = m_shooterEnc.GetPosition();

    m_shooterGoal = {m_shooterAngleGoal, 0_deg_per_s};

    m_shooterSetpoint = m_shooterProfile.Calculate(physical::kDt, m_shooterSetpoint, m_shooterGoal);

    double shooterOutput = m_shooterPID.Calculate(m_shooterPosition.value(), m_shooterSetpoint.position.value());
    double shooterFFOutput = m_shooterFeedforward.Calculate(m_shooterSetpoint.position, m_shooterSetpoint.velocity).value();

    m_angleShooterMotor.Set(shooterOutput + shooterFFOutput / 12);
}

void ShooterSubsystem::GoToAngle(units::degree_t shooterAngleGoal) {
    m_shooterAngleGoal = shooterAngleGoal;

    if(m_shooterAngleGoal > physical::kShooterMaxAngle) {m_shooterAngleGoal = physical::kShooterMaxAngle;}
    if(m_shooterAngleGoal < physical::kShooterMinAngle) {m_shooterAngleGoal = physical::kShooterMinAngle;}
}

void ShooterSubsystem::Spin(double speed) {
    m_topShooterMotor.Set(speed);
    m_shooterSpeed = speed;
}

bool ShooterSubsystem::AtSpeed() {
    return m_topShooterMotor.GetAppliedOutput() >= m_shooterSpeed;
}
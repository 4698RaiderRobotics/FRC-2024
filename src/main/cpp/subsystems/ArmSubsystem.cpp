// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "subsystems/ArmSubsystem.h"

#include <frc/DriverStation.h>
#include <frc/smartdashboard/SmartDashboard.h>

ArmSubsystem::ArmSubsystem() {
    ctre::phoenix6::configs::CANcoderConfiguration wristAbsoluteEncoderConfigs{};
    wristAbsoluteEncoderConfigs.MagnetSensor.SensorDirection = true;
    wristAbsoluteEncoderConfigs.MagnetSensor.MagnetOffset = physical::kWristAbsoluteOffset;
    m_wristEncoder.GetConfigurator().Apply(wristAbsoluteEncoderConfigs, 50_ms);

    m_wristMotor.GetConfigurator().Apply(ctre::phoenix6::configs::TalonFXConfiguration{});

    ctre::phoenix6::configs::TalonFXConfiguration wristConfigs{};
    wristConfigs.Slot0.GravityType = ctre::phoenix6::signals::GravityTypeValue::Arm_Cosine;
    wristConfigs.Slot0.kG = pidf::kWristG;
    wristConfigs.Slot0.kV = pidf::kWristV;
    wristConfigs.Slot0.kP = pidf::kWristP;
    wristConfigs.Slot0.kI = pidf::kWristI;
    wristConfigs.Slot0.kD = pidf::kWristD;
    wristConfigs.Feedback.FeedbackRemoteSensorID = m_wristEncoder.GetDeviceID();
    wristConfigs.Feedback.FeedbackSensorSource = ctre::phoenix6::signals::FeedbackSensorSourceValue::RemoteCANcoder;
    wristConfigs.MotionMagic.MotionMagicAcceleration = physical::kWristMaxSpeed;
    wristConfigs.MotionMagic.MotionMagicCruiseVelocity = physical::kWristMaxAcceleration;
    m_wristMotor.GetConfigurator().Apply(wristConfigs, 50_ms);


    ctre::phoenix6::configs::CANcoderConfiguration armAbsoluteEncoderConfigs{};
    armAbsoluteEncoderConfigs.MagnetSensor.SensorDirection = true;
    armAbsoluteEncoderConfigs.MagnetSensor.MagnetOffset = -0.031;
    m_armEncoder.GetConfigurator().Apply(armAbsoluteEncoderConfigs, 50_ms);

    m_armMotor.GetConfigurator().Apply(ctre::phoenix6::configs::TalonFXConfiguration{});

    ctre::phoenix6::configs::TalonFXConfiguration armConfigs{};
    armConfigs.MotorOutput.Inverted = true;
    m_armMotor.GetConfigurator().Apply(armConfigs, 50_ms);


    // wristPosReference.SetUpdateFrequency(50_Hz);
    // wristVelReference.SetUpdateFrequency(50_Hz);
    // armVelReference.SetUpdateFrequency(50_Hz);
    // armVel.SetUpdateFrequency(50_Hz);
};

// This method will be called once per scheduler run
void ArmSubsystem::Periodic() {

    // wristPos.Refresh();
    // wristVel.Refresh();
    // wristPosReference.Refresh();
    // wristVelReference.Refresh();

    // armPos.Refresh();
    // armVel.Refresh();
    // armPosReference.Refresh();
    // armVelReference.Refresh();

    m_wristAngle = m_wristEncoder.GetPosition().GetValueAsDouble() * 360_deg;
    phi = m_armEncoder.GetPosition().GetValueAsDouble() * 360_deg;

    m_armAngle = m_wristAngle - phi;

    frc::SmartDashboard::PutNumber("Wrist Angle", m_wristAngle.value());
    // frc::SmartDashboard::PutNumber("Phi", phi.value());
    frc::SmartDashboard::PutNumber("Arm Angle", m_armAngle.value());

    // frc::SmartDashboard::PutNumber("Wrist Velocity", wristVel.GetValueAsDouble());
    // frc::SmartDashboard::PutNumber("Wrist Motion Magic Pos", wristPosReference.GetValueAsDouble() * 360.0);
    // frc::SmartDashboard::PutNumber("Wrist Motion Magic Vel", wristVelReference.GetValueAsDouble());

    // frc::SmartDashboard::PutNumber("Arm Velocity", armVel.GetValueAsDouble() * 360 / 60 );
    // frc::SmartDashboard::PutNumber("Arm Setpoint Position", m_armSetpoint.position.value());
    // frc::SmartDashboard::PutNumber("Arm Setpoint Velocity", m_armSetpoint.velocity.value());


    if ( frc::DriverStation::IsDisabled() ) {
        m_armSetpoint.position = m_armAngle;
        m_armSetpoint.velocity = 0_deg_per_s;
        m_armGoal.position = m_armAngle;
        m_armGoal.velocity = 0_deg_per_s;

        m_wristGoal.position = m_wristAngle;
        m_wristGoal.velocity = 0_deg_per_s;

        return;
    }

    m_wristMotor.SetControl( m_wristPositionDC.WithPosition( m_wristGoal.position ) ); 
    
    m_armSetpoint = m_armProfile.Calculate(physical::kDt, m_armSetpoint, m_armGoal);

    double armOutput = m_armPID.Calculate( m_armAngle.value(), m_armSetpoint.position.value() );
    double armFeedforwardOut = m_armFeedforward.Calculate( m_armSetpoint.position, m_armSetpoint.velocity).value() 
                                + pidf::kArmWristG * units::math::cos(m_wristAngle).value();

    m_armMotor.Set( armOutput + armFeedforwardOut / 12.0 );
}

void ArmSubsystem::GoToArmAngle(units::degree_t armAngleGoal) {
    if(armAngleGoal > physical::kArmMaxAngle) armAngleGoal = physical::kArmMaxAngle;
    if(armAngleGoal < physical::kArmMinAngle) armAngleGoal = physical::kArmMinAngle;

    m_armGoal.position = armAngleGoal;
}

void ArmSubsystem::GoToWristAngle(units::degree_t wristAngleGoal) {
    if (wristAngleGoal > physical::kWristMaxAngle) wristAngleGoal = physical::kWristMaxAngle;
    if (wristAngleGoal < physical::kWristMinAngle) wristAngleGoal = physical::kWristMinAngle;

    m_wristGoal.position = wristAngleGoal;
}

void ArmSubsystem::NudgeArmAngle(units::degree_t deltaAngle) {
    m_armGoal.position += deltaAngle;
}

void ArmSubsystem::NudgeWristAngle(units::degree_t deltaAngle) {
    m_wristGoal.position += deltaAngle;
}

units::degree_t ArmSubsystem::GetArmAngle() {
    return m_armAngle;
}

units::degree_t ArmSubsystem::GetWristAngle() {
    return m_wristAngle;
}

bool ArmSubsystem::IsAtGoal( units::degree_t arm_tol ) {
    return units::math::abs(m_wristAngle - m_wristGoal.position) < arm_tol &&
           units::math::abs(m_armAngle - m_armGoal.position) < arm_tol;
}
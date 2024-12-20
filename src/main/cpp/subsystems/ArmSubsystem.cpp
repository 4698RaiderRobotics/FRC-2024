// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "subsystems/ArmSubsystem.h"

#include "DataLogger.h"

#include <frc/DriverStation.h>
#include <frc/Preferences.h>

#include <frc2/command/Commands.h>

#include "DeviceConstants.h"
#include "Constants.h"

ArmSubsystem::ArmSubsystem() :
    m_wristMotor{deviceIDs::kWristMotorID},
    m_wristEncoder{deviceIDs::kWristEncoderID},
    m_armMotor{deviceIDs::kArmMotorID},
    m_armEncoder{deviceIDs::kArmEncoderID},
    m_armPID{pidf::kArmP, pidf::kArmI, pidf::kArmD},
    m_armFeedforward{units::volt_t{pidf::kArmS}, units::volt_t{pidf::kArmG}, 
                     units::unit_t<frc::ArmFeedforward::kv_unit> {pidf::kArmV}, 
                     units::unit_t<frc::ArmFeedforward::ka_unit> {pidf::kArmA} },
    m_armProfile{{pidf::kArmMaxSpeed, pidf::kArmMaxAcceleration}}

{
    frc::Preferences::InitDouble("ArmOffset", 0.0);
    frc::Preferences::InitDouble("WristOffset", 0.0);

    ctre::phoenix6::configs::CANcoderConfiguration wristAbsoluteEncoderConfigs{};
    wristAbsoluteEncoderConfigs.MagnetSensor.SensorDirection = true;
    wristAbsoluteEncoderConfigs.MagnetSensor.MagnetOffset = frc::Preferences::GetDouble("WristOffset");
    m_wristEncoder.GetConfigurator().Apply(wristAbsoluteEncoderConfigs, 50_ms);

    m_wristMotor.GetConfigurator().Apply(ctre::phoenix6::configs::TalonFXConfiguration{});

    ctre::phoenix6::configs::TalonFXConfiguration wristConfigs{};
    wristConfigs.Slot0.GravityType = ctre::phoenix6::signals::GravityTypeValue::Arm_Cosine;
    wristConfigs.Slot0.kS = pidf::kWristS;
    wristConfigs.Slot0.kG = pidf::kWristG;
    wristConfigs.Slot0.kV = pidf::kWristV;
    wristConfigs.Slot0.kA = pidf::kWristA;
    wristConfigs.Slot0.kP = pidf::kWristP;
    wristConfigs.Slot0.kI = pidf::kWristI;
    wristConfigs.Slot0.kD = pidf::kWristD;
    wristConfigs.Feedback.FeedbackRemoteSensorID = m_wristEncoder.GetDeviceID();
    wristConfigs.Feedback.FeedbackSensorSource = ctre::phoenix6::signals::FeedbackSensorSourceValue::RemoteCANcoder;
    wristConfigs.Feedback.RotorToSensorRatio = kWristGearRatio;
    wristConfigs.MotionMagic.MotionMagicAcceleration = pidf::kWristMaxAcceleration;
    wristConfigs.MotionMagic.MotionMagicCruiseVelocity = pidf::kWristMaxSpeed;
    m_wristMotor.GetConfigurator().Apply(wristConfigs, 50_ms);


    ctre::phoenix6::configs::CANcoderConfiguration armAbsoluteEncoderConfigs{};
    armAbsoluteEncoderConfigs.MagnetSensor.SensorDirection = true;
    armAbsoluteEncoderConfigs.MagnetSensor.MagnetOffset = frc::Preferences::GetDouble("ArmOffset");
    m_armEncoder.GetConfigurator().Apply(armAbsoluteEncoderConfigs, 50_ms);

    m_armMotor.GetConfigurator().Apply(ctre::phoenix6::configs::TalonFXConfiguration{});

    ctre::phoenix6::configs::TalonFXConfiguration armConfigs{};
    armConfigs.MotorOutput.Inverted = true;
    m_armMotor.GetConfigurator().Apply(armConfigs, 50_ms);
};

void ArmSubsystem::Periodic() {

    m_wristAngle = m_wristEncoder.GetPosition().GetValueAsDouble() * 360_deg;
    phi = m_armEncoder.GetPosition().GetValueAsDouble() * 360_deg;

    m_armAngle = m_wristAngle - phi;

    DataLogger::Log( "ArmSubsys/Arm Angle", m_armAngle.value(), true );
    DataLogger::Log( "ArmSubsys/Wrist Angle", m_wristAngle.value(), true );

    if ( frc::DriverStation::IsDisabled() ) {
        m_armSetpoint.position = m_armAngle;
        m_armSetpoint.velocity = 0_deg_per_s;
        m_armGoal.position = m_armAngle;
        m_armGoal.velocity = 0_deg_per_s;

        m_wristGoal.position = m_wristAngle;
        m_wristGoal.velocity = 0_deg_per_s;
        return;
    }

    DataLogger::Log( "ArmSubsys/Arm Goal", m_armGoal.position.value(), true );
    DataLogger::Log( "ArmSubsys/Wrist Goal", m_wristGoal.position.value(), true );
    DataLogger::Log( "ArmSubsys/IsAtGoal", AtGoal() );
    DataLogger::Log( "ArmSubsys/Arm Mtr Voltage", m_armMotor.GetMotorVoltage().GetValueAsDouble() );
    DataLogger::Log( "ArmSubsys/Arm Mtr Current", m_armMotor.GetSupplyCurrent().GetValueAsDouble() );
    DataLogger::Log( "ArmSubsys/Wrist Mtr Voltage", m_wristMotor.GetMotorVoltage().GetValueAsDouble() );
    DataLogger::Log( "ArmSubsys/Wrist Mtr Current", m_wristMotor.GetSupplyCurrent().GetValueAsDouble() );

    m_wristMotor.SetControl( m_wristPositionDC.WithPosition( m_wristGoal.position ) ); 
    
    m_armSetpoint = m_armProfile.Calculate(physical::kDt, m_armSetpoint, m_armGoal);

    double armOutput = m_armPID.Calculate( m_armAngle.value(), m_armSetpoint.position.value() );
    double armFeedforwardOut = m_armFeedforward.Calculate( m_armSetpoint.position, m_armSetpoint.velocity).value() 
                                + pidf::kArmWristG * units::math::cos(m_wristAngle).value();

    DataLogger::Log( "ArmSubsys/Arm PID output", armOutput );
    DataLogger::Log( "ArmSubsys/Arm FF Output", armFeedforwardOut/12.0 );

    m_armMotor.Set( armOutput + armFeedforwardOut / 12.0 );
}

void ArmSubsystem::SetArmGoal(units::degree_t armAngleGoal) {
    if(armAngleGoal > physical::kArmMaxAngle) armAngleGoal = physical::kArmMaxAngle;
    if(armAngleGoal < physical::kArmMinAngle) armAngleGoal = physical::kArmMinAngle;

    m_armGoal.position = armAngleGoal;
}

void ArmSubsystem::SetWristGoal(units::degree_t wristAngleGoal) {
    if (wristAngleGoal > physical::kWristMaxAngle) wristAngleGoal = physical::kWristMaxAngle;
    if (wristAngleGoal < physical::kWristMinAngle) wristAngleGoal = physical::kWristMinAngle;

    m_wristGoal.position = wristAngleGoal;
}

void ArmSubsystem::NudgeArmAngle(units::degree_t deltaAngle) {
    SetArmGoal( m_armGoal.position + deltaAngle );
}

void ArmSubsystem::NudgeWristAngle(units::degree_t deltaAngle) {
    SetWristGoal( m_wristGoal.position + deltaAngle );
}

units::degree_t ArmSubsystem::GetArmAngle() {
    return m_armAngle;
}

units::degree_t ArmSubsystem::GetWristAngle() {
    return m_wristAngle;
}

bool ArmSubsystem::AtGoal( units::degree_t arm_tol ) {
    return units::math::abs(m_wristAngle - m_wristGoal.position) < arm_tol &&
           units::math::abs(m_armAngle - m_armGoal.position) < arm_tol;
}

void ArmSubsystem::UpdateEncoderOffsets() {
    ctre::phoenix6::configs::CANcoderConfiguration armAbsoluteEncoderConfigs{};
    m_armEncoder.GetConfigurator().Refresh( armAbsoluteEncoderConfigs );

    double offset = armAbsoluteEncoderConfigs.MagnetSensor.MagnetOffset - m_armEncoder.GetAbsolutePosition().GetValueAsDouble();

    frc::Preferences::SetDouble("ArmOffset", offset);

    armAbsoluteEncoderConfigs.MagnetSensor.MagnetOffset = offset;
    m_armEncoder.GetConfigurator().Apply(armAbsoluteEncoderConfigs, 50_ms);


    ctre::phoenix6::configs::CANcoderConfiguration wristAbsoluteEncoderConfigs{};
    m_wristEncoder.GetConfigurator().Refresh( wristAbsoluteEncoderConfigs );

    offset = wristAbsoluteEncoderConfigs.MagnetSensor.MagnetOffset - m_wristEncoder.GetAbsolutePosition().GetValueAsDouble();

    frc::Preferences::SetDouble("WristOffset", offset);

    wristAbsoluteEncoderConfigs.MagnetSensor.MagnetOffset = offset;
    m_wristEncoder.GetConfigurator().Apply(wristAbsoluteEncoderConfigs, 50_ms);

}

frc2::CommandPtr ArmSubsystem::MoveJoints( units::degree_t armAngle, units::degree_t wristAngle ) {
    return frc2::cmd::Sequence(
        RunOnce( [this, armAngle, wristAngle] { 
            SetArmGoal( armAngle );
            SetWristGoal( wristAngle );
        }),
        frc2::cmd::WaitUntil( [this] { return AtGoal(); } ).WithTimeout( 3_s )
    ).WithName( "MoveJoints" );
}

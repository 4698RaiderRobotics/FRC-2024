// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <units/angle.h>

#include <frc2/command/SubsystemBase.h>

#include <frc/controller/ArmFeedforward.h>
#include <frc/controller/PIDController.h>
#include <frc/trajectory/TrapezoidProfile.h>

#include <ctre/phoenix6/CANcoder.hpp>
#include <ctre/phoenix6/TalonFX.hpp>

class ArmSubsystem : public frc2::SubsystemBase {
 public:
  ArmSubsystem();

  /**
   * Will be called periodically whenever the CommandScheduler runs.
   */
  void Periodic() override;

  void GoToArmAngle(units::degree_t armAngleGoal);

  void GoToWristAngle(units::degree_t wristAngleGoal);

  void NudgeWristAngle(units::degree_t deltaAngle);

  void NudgeArmAngle(units::degree_t deltaAngle);

  units::degree_t GetArmAngle();

  units::degree_t GetWristAngle();

  bool IsAtGoal(units::degree_t arm_tol = 6_deg);

  void UpdateEncoderOffsets();
  
 private:
   const double kWristGearRatio = 36.0 * 42 / 38;

  ctre::phoenix6::hardware::TalonFX m_wristMotor;
  ctre::phoenix6::hardware::CANcoder m_wristEncoder;
  ctre::phoenix6::controls::MotionMagicDutyCycle m_wristPositionDC{0_deg};
  ctre::phoenix6::StatusSignal<units::turn_t> wristPos = m_wristMotor.GetPosition();
  ctre::phoenix6::StatusSignal<units::turns_per_second_t> wristVel = m_wristMotor.GetVelocity();
  ctre::phoenix6::StatusSignal<double> wristPosReference = m_wristMotor.GetClosedLoopReference();
  ctre::phoenix6::StatusSignal<double> wristVelReference = m_wristMotor.GetClosedLoopReferenceSlope();

  frc::TrapezoidProfile<units::degrees>::State m_wristGoal;

  units::degree_t m_wristAngle;
  


  ctre::phoenix6::hardware::TalonFX m_armMotor;
  ctre::phoenix6::hardware::CANcoder m_armEncoder;
  ctre::phoenix6::controls::MotionMagicDutyCycle m_armPositionDC{0_deg};
  ctre::phoenix6::StatusSignal<units::turn_t> armPos = m_armMotor.GetPosition();
  ctre::phoenix6::StatusSignal<units::turns_per_second_t> armVel = m_armMotor.GetRotorVelocity();
  ctre::phoenix6::StatusSignal<double> armPosReference = m_armMotor.GetClosedLoopReference();
  ctre::phoenix6::StatusSignal<double> armVelReference = m_armMotor.GetClosedLoopReferenceSlope();

  frc::PIDController m_armPID;
  frc::ArmFeedforward m_armFeedforward;
  
  frc::TrapezoidProfile<units::degrees> m_armProfile;
  frc::TrapezoidProfile<units::degrees>::State m_armGoal;
  frc::TrapezoidProfile<units::degrees>::State m_armSetpoint{};

  units::degree_t m_armAngle;
  units::degree_t phi;
};

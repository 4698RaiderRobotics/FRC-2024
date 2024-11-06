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

  void Periodic() override;

  units::degree_t GetArmAngle();
  units::degree_t GetWristAngle();

  void SetArmGoal(units::degree_t armAngleGoal);
  void SetWristGoal(units::degree_t wristAngleGoal);

  void NudgeWristAngle(units::degree_t deltaAngle);
  void NudgeArmAngle(units::degree_t deltaAngle);

  bool AtGoal(units::degree_t angle_tol = 6_deg);

  void UpdateEncoderOffsets();

/**
   *  Create a command to move the arm and wrist to the specified angles.
   */
  [[nodiscard]]
  frc2::CommandPtr MoveJoints( units::degree_t armAngle, units::degree_t wristAngle );

 private:
  const double kWristGearRatio = 20.0 * 42 / 38;

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

  frc::PIDController m_armPID;
  frc::ArmFeedforward m_armFeedforward;
  
  frc::TrapezoidProfile<units::degrees> m_armProfile;
  frc::TrapezoidProfile<units::degrees>::State m_armGoal;
  frc::TrapezoidProfile<units::degrees>::State m_armSetpoint;

  units::degree_t m_armAngle;
  units::degree_t phi;
};

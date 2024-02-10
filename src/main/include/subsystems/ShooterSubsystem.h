// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc2/command/SubsystemBase.h>

#include <frc/controller/ArmFeedforward.h>
#include <frc/controller/PIDController.h>
#include <frc/trajectory/TrapezoidProfile.h>

#include <rev/CANSparkFlex.h>
#include <rev/CANSparkMax.h>

#include "Constants.h"
#include "AbsoluteEncoder.h"

class ShooterSubsystem : public frc2::SubsystemBase {
 public:
  ShooterSubsystem();

  /**
   * Will be called periodically whenever the CommandScheduler runs.
   */
  void Periodic() override;

  // Changes the setpoint for the shooter angle
  void GoToAngle(units::degree_t shooterAngleGoal);

  // Spins the shooter at a speed
  void Spin(double speed);

  // Checks if the shooter is at the right speed
  bool AtSpeed();

 private:
  // Components (e.g. motor controllers and sensors) should generally be
  // declared private and exposed only through public methods.
  rev::CANSparkFlex m_topShooterMotor{deviceIDs::kTopShooterID, rev::CANSparkFlex::MotorType::kBrushless};
  rev::CANSparkFlex m_bottomShooterMotor{deviceIDs::kBottomShooterID, rev::CANSparkFlex::MotorType::kBrushless};

  rev::CANSparkMax m_angleShooterMotor{deviceIDs::kShooterAngleID, rev::CANSparkMax::MotorType::kBrushless};

  AbsoluteEncoder m_shooterEnc{deviceIDs::kShooterEncoderID, physical::kShooterAbsoluteOffset};
  

  frc::PIDController m_shooterPID{pidf::kShooterP, pidf::kShooterI, pidf::kShooterD};
  frc::ArmFeedforward m_shooterFeedforward{units::volt_t{pidf::kShooterS}, units::volt_t{pidf::kShooterG}, 
                                        units::unit_t<frc::ArmFeedforward::kv_unit> {pidf::kShooterV}, 
                                        units::unit_t<frc::ArmFeedforward::ka_unit> {pidf::kShooterA}};
  
  frc::TrapezoidProfile<units::degrees> m_shooterProfile{{physical::kShooterMaxSpeed, physical::kShooterMaxAcceleration}};
  frc::TrapezoidProfile<units::degrees>::State m_shooterGoal;
  frc::TrapezoidProfile<units::degrees>::State m_shooterSetpoint{};

  units::degree_t m_shooterAngleGoal;
  units::degree_t m_shooterPosition;


  double m_shooterSpeed;
};

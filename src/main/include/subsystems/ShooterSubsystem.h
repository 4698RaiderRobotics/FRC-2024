// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <vector>

#include <frc2/command/SubsystemBase.h>

#include <frc/controller/ArmFeedforward.h>
#include <frc/controller/PIDController.h>
#include <frc/trajectory/TrapezoidProfile.h>

#include <rev/CANSparkFlex.h>
#include <rev/CANSparkMax.h>
#include <ctre/phoenix6/CANcoder.hpp>

#include "Constants.h"
#include "LUT.h"

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
  void Spin(units::revolutions_per_minute_t speed);

  units::degree_t GetAngle();

  void Nudge( units::degree_t deltaAngle );

    // Checks if the shooter is at the right speed
  bool IsAtSpeed();

    // Checks if the shooter is at the right angle
  bool IsAtAngle();

    // Checks if the shooter is at the right speed and angle
  bool IsAtGoal();

  void UpdateEncoderOffset();

  units::degree_t GetShooter_ArmAngle()
    { return arm_lut.lookup( m_shooterPosition.value() ) * 1_deg; }
  units::meter_t GetShooter_ElevatorHeight()
    { return elev_lut.lookup( m_shooterPosition.value() ) * 1_m; }


 private:
  // Components (e.g. motor controllers and sensors) should generally be
  // declared private and exposed only through public methods.
  rev::CANSparkFlex m_rightShooterMotor{deviceIDs::kRightShooterID, rev::CANSparkFlex::MotorType::kBrushless};
  rev::CANSparkFlex m_leftShooterMotor{deviceIDs::kLeftShooterID, rev::CANSparkFlex::MotorType::kBrushless};

  rev::SparkPIDController m_speedPID = m_rightShooterMotor.GetPIDController();

  rev::SparkRelativeEncoder m_rightEncoder = m_rightShooterMotor.GetEncoder();

  rev::CANSparkMax m_angleShooterMotor{deviceIDs::kShooterAngleID, rev::CANSparkMax::MotorType::kBrushless};

  ctre::phoenix6::hardware::CANcoder m_shooterAngleEncoder{deviceIDs::kShooterEncoderID};
  

  frc::PIDController m_shooterPID{pidf::kShooterP, pidf::kShooterI, pidf::kShooterD};
  frc::ArmFeedforward m_shooterFeedforward{units::volt_t{pidf::kShooterS}, units::volt_t{pidf::kShooterG}, 
                                        units::unit_t<frc::ArmFeedforward::kv_unit> {pidf::kShooterV}, 
                                        units::unit_t<frc::ArmFeedforward::ka_unit> {pidf::kShooterA}};
  
  frc::TrapezoidProfile<units::degrees> m_shooterProfile{{physical::kShooterMaxSpeed, physical::kShooterMaxAcceleration}};
  frc::TrapezoidProfile<units::degrees>::State m_shooterGoal;
  frc::TrapezoidProfile<units::degrees>::State m_shooterSetpoint{};

  units::degree_t m_shooterAngleGoal;
  units::degree_t m_shooterPosition;

  units::revolutions_per_minute_t m_speed;

  double m_shooterSpeed;

    // Arm and Wrist lookup tables: maps shooter angle to arm and wrist angles
  // LUT arm_lut{ {25.0, 45.0}, {145.0, 180.0} };
  // LUT wrist_lut{ {25.0, 45.0}, {150.0, 130.0} };
  // LUT arm_lut{ {26.0, 33.5, 45.0}, {158.0, 169, 177.5} };
//  LUT arm_lut{ {18.0, 26.0, 33.5, 45.0}, {159.0, 155.0, 166, 174.5} };
  // LUT arm_lut{ {18.0, 26.0, 33.5, 45.0}, {156.0, 155.0, 166, 174.5} };
  LUT arm_lut{ {15.0, 20.0, 25.0, 30.0, 35.0, 45.0, 90.0}, {178.0, 178.0, 178, 178, 178, 178, 178} };
//  LUT wrist_lut{ {18.0, 26.0, 33.5, 45.0}, {153.0, 152.3, 142.5, 136.3} };

    // Elevator Height (meters) versus shooting angle.
  LUT elev_lut{ {18.0,  20.0,  25.0,  30.0,  35.0,  40.0,  45.0, 90.0}, 
                {0.103, 0.102, 0.087, 0.061, 0.035, 0.025, 0.007, 0.0} };
};

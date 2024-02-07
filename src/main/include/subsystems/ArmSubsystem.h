// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc2/command/SubsystemBase.h>

#include <frc/controller/ArmFeedforward.h>
#include <frc/controller/PIDController.h>
#include <frc/trajectory/TrapezoidProfile.h>

#include "Constants.h"

class ArmSubsystem : public frc2::SubsystemBase {
 public:
  ArmSubsystem();

  /**
   * Will be called periodically whenever the CommandScheduler runs.
   */
  void Periodic() override;

  void GoToAngle(units::degree_t armAngleGoal);

 private:
  // Components (e.g. motor controllers and sensors) should generally be
  // declared private and exposed only through public methods.

  frc::PIDController m_armPID{pidf::kArmP, pidf::kArmI, pidf::kArmD};
  frc::ArmFeedforward m_armFeedforward{units::volt_t{pidf::kArmS}, units::volt_t{pidf::kArmG}, 
                                        units::unit_t<frc::ArmFeedforward::kv_unit> {pidf::kArmV}, 
                                        units::unit_t<frc::ArmFeedforward::ka_unit> {pidf::kArmA}};
  
  frc::TrapezoidProfile<units::degrees> m_armProfile{{physical::kArmMaxSpeed, physical::kArmMaxAcceleration}};
  frc::TrapezoidProfile<units::degrees>::State m_armGoal;
  frc::TrapezoidProfile<units::degrees>::State m_armSetpoint{};

  units::degree_t m_armAngleGoal;
};

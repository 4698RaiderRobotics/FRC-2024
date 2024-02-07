// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc2/command/SubsystemBase.h>

#include <frc/controller/ElevatorFeedforward.h>
#include <frc/controller/PIDController.h>
#include <frc/trajectory/TrapezoidProfile.h>

#include "Constants.h"

class ElevatorSubsystem : public frc2::SubsystemBase {
 public:
  ElevatorSubsystem();

  /**
   * Will be called periodically whenever the CommandScheduler runs.
   */
  void Periodic() override;

  void GoToHeight(units::meter_t elevatorHeightGoal);

 private:
  // Components (e.g. motor controllers and sensors) should generally be
  // declared private and exposed only through public methods.

  frc::PIDController m_elevatorPID{pidf::kElevatorP, pidf::kElevatorI, pidf::kElevatorD};
  frc::ElevatorFeedforward m_elevatorFeedforward{units::volt_t{pidf::kElevatorS}, units::volt_t{pidf::kElevatorG}, 
                                        units::unit_t<frc::ElevatorFeedforward::kv_unit> {pidf::kElevatorV}, 
                                        units::unit_t<frc::ElevatorFeedforward::ka_unit> {pidf::kElevatorA}};
  
  frc::TrapezoidProfile<units::meters> m_elevatorProfile{{physical::kElevatorMaxSpeed, physical::kElevatorMaxAcceleration}};
  frc::TrapezoidProfile<units::meters>::State m_elevatorGoal;
  frc::TrapezoidProfile<units::meters>::State m_elevatorSetpoint{};

  units::meter_t m_elevatorHeightGoal = 0_m;
};

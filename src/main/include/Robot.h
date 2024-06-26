// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <optional>

#include <frc/TimedRobot.h>
#include <frc2/command/CommandPtr.h>
#include <frc/PowerDistribution.h>
#include <frc/smartdashboard/SmartDashboard.h>

#include "LoggedRobot.h"
#include "RobotContainer.h"

class Robot : public LoggedRobot {
 public:
  void RobotInit() override;
  void RobotPeriodic() override;
  void DisabledInit() override;
  void DisabledPeriodic() override;
  void DisabledExit() override;
  void AutonomousInit() override;
  void AutonomousPeriodic() override;
  void AutonomousExit() override;
  void TeleopInit() override;
  void TeleopPeriodic() override;
  void TeleopExit() override;
  void TestInit() override;
  void TestPeriodic() override;
  void TestExit() override;

 private:
  frc2::Command* m_autonomousCommand;

  RobotContainer m_container;

//  frc::PowerDistribution m_pdp{ 1, frc::PowerDistribution::ModuleType::kRev };

  std::string_view kArmOffsetKey = "ArmOffset";
  std::string_view kWristOffsetKey = "WristOffset";
  std::string_view kShooterOffsetKey = "ShooterOffset";

};

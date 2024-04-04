// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "Robot.h"

#include "DataLogger.h"

#include <frc2/command/CommandScheduler.h>
#include <frc/Preferences.h>

void Robot::RobotInit() {
    LoggedRobot::RobotInit();
  
  frc::SmartDashboard::PutBoolean("Update Arm Preferences", false);
  frc::SmartDashboard::PutBoolean("Update Shooter Preferences", false);

}

void Robot::RobotPeriodic() {
  frc2::CommandScheduler::GetInstance().Run();

  LoggedRobot::RobotPeriodic();
}

void Robot::DisabledInit() {}

void Robot::DisabledPeriodic() {
  if(frc::SmartDashboard::GetBoolean("Update Arm Preferences", false)) {
    m_container.m_arm.UpdateEncoderOffsets();
    frc::SmartDashboard::PutBoolean("Update Arm Preferences", false);
  }
  if(frc::SmartDashboard::GetBoolean("Update Shooter Preferences", false)) {
    m_container.m_shooter.UpdateEncoderOffset();
    frc::SmartDashboard::PutBoolean("Update Shooter Preferences", false);
  }
}

void Robot::DisabledExit() {}

void Robot::AutonomousInit() {
  m_autonomousCommand = m_container.GetAutonomousCommand();

  if (m_autonomousCommand != nullptr ) {
    m_autonomousCommand->Schedule();
  }
}

void Robot::AutonomousPeriodic() {}

void Robot::AutonomousExit() {}

void Robot::TeleopInit() {
  if (m_autonomousCommand != nullptr) {
    m_autonomousCommand->Cancel();
  }
}

void Robot::TeleopPeriodic() {}

void Robot::TeleopExit() {}

void Robot::TestInit() {
  frc2::CommandScheduler::GetInstance().CancelAll();
}

void Robot::TestPeriodic() {}

void Robot::TestExit() {}

#ifndef RUNNING_FRC_TESTS
int main() {
  return frc::StartRobot<Robot>();
}
#endif

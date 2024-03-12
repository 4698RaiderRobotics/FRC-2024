// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "Robot.h"

#include "DataLogger.h"

#include <frc2/command/CommandScheduler.h>

void Robot::RobotInit() {}

void Robot::RobotPeriodic() {
  frc2::CommandScheduler::GetInstance().Run();

  DataLogger::GetInstance().SendNT("PDP/Bus Voltage", m_pdp.GetVoltage());
  DataLogger::GetInstance().SendNT("PDP/Total Current", m_pdp.GetTotalCurrent());
  DataLogger::GetInstance().SendNT("PDP/Temperature", m_pdp.GetTemperature());
  DataLogger::GetInstance().SendNT("PDP/Total Power", m_pdp.GetTotalPower());
  DataLogger::GetInstance().SendNT("PDP/Brown Out", (bool) m_pdp.GetFaults().Brownout);
}

void Robot::DisabledInit() {}

void Robot::DisabledPeriodic() {}

void Robot::DisabledExit() {}

void Robot::AutonomousInit() {
  m_autonomousCommand = m_container.GetAutonomousCommand();

  if (m_autonomousCommand) {
    m_autonomousCommand->Schedule();
  }
}

void Robot::AutonomousPeriodic() {}

void Robot::AutonomousExit() {}

void Robot::TeleopInit() {
  if (m_autonomousCommand) {
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

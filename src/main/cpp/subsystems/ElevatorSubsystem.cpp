// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "subsystems/ElevatorSubsystem.h"

ElevatorSubsystem::ElevatorSubsystem() = default;

// This method will be called once per scheduler run
void ElevatorSubsystem::Periodic() {
    m_elevatorGoal = {m_elevatorHeightGoal, 0_mps};
}

void ElevatorSubsystem::GoToHeight(units::meter_t elevatorHeightGoal) {
    m_elevatorHeightGoal = elevatorHeightGoal;

    if(m_elevatorHeightGoal > physical::kElevatorMaxHeight) {m_elevatorHeightGoal = physical::kElevatorMaxHeight;}
    if(m_elevatorHeightGoal < physical::kElevatorMinHeight) {m_elevatorHeightGoal = physical::kElevatorMinHeight;}
}
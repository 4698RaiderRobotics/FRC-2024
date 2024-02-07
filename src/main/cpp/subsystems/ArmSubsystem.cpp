// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "subsystems/ArmSubsystem.h"

ArmSubsystem::ArmSubsystem() = default;

// This method will be called once per scheduler run
void ArmSubsystem::Periodic() {
    
}

void ArmSubsystem::GoToAngle(units::degree_t armAngleGoal) {
    m_armAngleGoal = armAngleGoal;

    if(m_armAngleGoal > physical::kArmMaxAngle) {m_armAngleGoal = physical::kArmMaxAngle;}
    if(m_armAngleGoal < physical::kArmMinAngle) {m_armAngleGoal = physical::kArmMinAngle;}
}
// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include <frc/DriverStation.h>

#include "commands/StageNoteInShooter.h"

#include "subsystems/IntakeSubsystem.h"
#include "subsystems/ShooterSubsystem.h"
#include "subsystems/ArmSubsystem.h"
#include "subsystems/ElevatorSubsystem.h"

#include "DataLogger.h"

StageNoteInShooter::StageNoteInShooter( ShooterSubsystem* shooter, 
                      IntakeSubsystem* intake, ArmSubsystem* arm, ElevatorSubsystem* elev ) :
                      m_shooter{shooter}, m_intake{intake}, m_arm{arm}, m_elev{elev}
{
  SetName( "StageNoteInShooter" );

  AddRequirements({m_shooter, m_intake, m_arm, m_elev});
}

// Called when the command is initially scheduled.
void StageNoteInShooter::Init() {

    // Start moving to the correct wrist positions.
      m_arm->GoToWristAngle( 180_deg - m_shooter->GetAngle() );

}

// Called repeatedly when this Command is scheduled to run
void StageNoteInShooter::Execute() {
  // double t_yaw, t_pitch;

  units::degree_t arm_tolerance;

  arm_tolerance = 5_deg;

    if( m_arm->GetWristAngle() > 80_deg ) {
      m_arm->GoToArmAngle( m_shooter->GetShooter_ArmAngle() );
      m_elev->GoToHeight( m_shooter->GetShooter_ElevatorHeight() );
    }
    m_arm->GoToWristAngle( 180_deg - m_shooter->GetAngle( ) );
      

    // DataLogger::GetInstance().SendNT( "StageNoteInShooter/Shooter Angle", m_shooter->GetAngle().value() );
    // DataLogger::GetInstance().SendNT( "StageNoteInShooter/Arm Angle Goal", m_shooter->GetShooter_ArmAngle().value() );
    // DataLogger::GetInstance().SendNT( "StageNoteInShooter/Arm Angle", m_arm->GetArmAngle().value() );
    // DataLogger::GetInstance().SendNT( "StageNoteInShooter/Wrist Angle Goal", 180 - m_shooter->GetAngle().value() );
    // DataLogger::GetInstance().SendNT( "StageNoteInShooter/Wrist Angle", m_arm->GetWristAngle().value() );
    // DataLogger::GetInstance().SendNT( "StageNoteInShooter/Elevator Height", m_elev->GetHeight().value() );
    // DataLogger::GetInstance().SendNT( "StageNoteInShooter/Arm IsAtGoal", m_arm->IsAtGoal( arm_tolerance ) );
    // DataLogger::GetInstance().SendNT( "StageNoteInShooter/Shooter IsAtAngle", m_shooter->IsAtAngle() );
    // DataLogger::GetInstance().SendNT( "StageNoteInShooter/Elevator IsAtGoal", m_elev->IsAtGoal() );

}

// Called once the command ends or is interrupted.
void StageNoteInShooter::Ending(bool interrupted) {

}

// Returns true when the command should end.
bool StageNoteInShooter::IsFinished() {

  return m_shooter->IsAtAngle() && m_arm->IsAtGoal() && m_elev->IsAtGoal();
}

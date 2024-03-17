// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "commands/GoToRestPosition.h"
#include "commands/ChangeWristAngle.h"

GoToRestPosition::GoToRestPosition(ArmSubsystem *arm, ElevatorSubsystem* elev, IntakeSubsystem *intake)
 : m_arm{arm}, m_elev{elev}, m_intake{intake} {
  SetName( "GoToRestPosition" );
  // Use addRequirements() here to declare subsystem dependencies.
  AddRequirements({arm});
}

// Called when the command is initially scheduled.
void GoToRestPosition::Init() {
  fmt::print( "GoToRestPosition::Initialize() -- hasNote ({})\n", m_intake->HasNote() );
  m_elev->GoToHeight( 0_in );
  // m_arm->GoToArmAngle(physical::kArmPassiveAngle);
  // m_arm->GoToWristAngle(physical::kWristPassiveAngle);

}

// Called repeatedly when this Command is scheduled to run
void GoToRestPosition::Execute() {

  if( m_elev->IsAtGoal() ) {
    m_arm->GoToArmAngle(physical::kArmPassiveAngle);
    if( m_arm->GetArmAngle() < 150_deg ) {
        m_arm->GoToWristAngle(physical::kWristPassiveAngle);
    } else {
      if(m_intake->HasNote() ) {
        m_arm->GoToWristAngle( 130_deg );
      } else {
        m_arm->GoToWristAngle(physical::kWristPassiveAngle);
      }
    }
  }
}

// Called once the command ends or is interrupted.
void GoToRestPosition::Ending(bool interrupted) {}

// Returns true when the command should end.
bool GoToRestPosition::IsFinished() {
  return m_arm->IsAtGoal() && m_elev->IsAtGoal();
}

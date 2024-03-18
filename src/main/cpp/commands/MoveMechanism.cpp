// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "commands/MoveMechanism.h"

MoveMechanism::MoveMechanism( ArmSubsystem *arm, ElevatorSubsystem* elev, 
                              units::degree_t arm_pos, units::degree_t wrist_pos, units::meter_t height )
 : m_arm{arm}, m_elev{elev}, m_arm_target{arm_pos}, m_wrist_target{wrist_pos}, m_height_target{ height } {
  SetName( "MoveMechanism" );
  // Use addRequirements() here to declare subsystem dependencies.
  AddRequirements({arm, elev});
}

// Called when the command is initially scheduled.
void MoveMechanism::Init() {

    // We need to safely move the mechanism from it's current position
    // to the target position.
    // The constraints are as follows:
    //
    // If the elevator is to be moved up or down then:
    //    The wrist must be at 90_deg while elevator height is > 4_in
    //    The arm must be at 75_deg  while elevator height is > 4_in
    //
    // As arm is moved from (<75_deg) to (>120_deg) then:
    //    The wrist must be at 35_deg.

    // Check if the elevator needs to move through the crash point.
  if( m_elev->GetHeight() > 4_in && m_height_target < 4_in ) {
    m_elev_going_up = false;
    m_elev_going_down = true;
    m_elev_is_done = false;
  } else if( m_elev->GetHeight() < 4_in && m_height_target > 4_in ) {
    m_elev_going_up = true;
    m_elev_going_down = false;
    m_elev_is_done = false;
  } else {
      // Elevator doesn't need to move far.
    m_elev_going_up = false;
    m_elev_going_down = false;
    m_elev_is_done = true;
  }

    // Check if the arm need to move through the crash point.
  if( m_arm->GetArmAngle() < 80_deg && m_arm_target > 120_deg ) {
    m_arm_going_back = true;
    m_arm_going_forward = false;
    m_wrist_retracted = false;
  } else if( m_arm->GetArmAngle() > 120_deg && m_arm_target < 80_deg ) {
    m_arm_going_back = false;
    m_arm_going_forward = true;
    m_wrist_retracted = false;
  } else {
      // Arm doesn't need to move far.
    m_arm_going_back = false;
    m_arm_going_forward = false;
    m_wrist_retracted = true;
  }
}

// Called repeatedly when this Command is scheduled to run
void MoveMechanism::Execute() {

  if( m_elev_going_up ) {
      // The elevator needs to go up
    if( m_arm_going_forward ) {
      // The arm is back and the elevator needs to go up
      // start with the arm / wrist move forward.
      if( FlipArmForward() ) {
          m_elev->GoToHeight( m_height_target );
      }
    } else {
        // Should be safe to move everything at once.
      m_elev->GoToHeight( m_height_target );
      m_arm->GoToArmAngle( m_arm_target );
      m_arm->GoToWristAngle( m_wrist_target );
    }
  } else if( m_elev_going_down ) {
      // If the elevator needs to come down
    if( m_arm_going_back ) {
      // The elevator is up and the arm needs to go back
      // start with the elevator move down.
      if( !m_elev_is_done ) {
        m_elev->GoToHeight( m_height_target );
        m_arm->GoToArmAngle( 75_deg );
        m_arm->GoToWristAngle( 90_deg );
        if( m_elev->GetHeight() - 4_in < 0_in ) {
          // We got the elevator low enough. Retracting the wrist
          m_elev_is_done = true;
        }
      } else {
        FlipArmBackward();
      }
    } else {
        // Should be safe to move everything at once.
      m_elev->GoToHeight( m_height_target );
      m_arm->GoToArmAngle( m_arm_target );
      m_arm->GoToWristAngle( m_wrist_target );
    }
  } else if( m_arm_going_forward ) {
      // Transitioning from backward to forward with the arm.
    FlipArmForward();
    m_elev->GoToHeight( m_height_target );
  } else if( m_arm_going_back ) {
      // Transitioning from forward to backward with the arm.
    FlipArmBackward();
    m_elev->GoToHeight( m_height_target );
  } else {
      // Should be safe to move everything at once.
    m_elev->GoToHeight( m_height_target );
    m_arm->GoToArmAngle( m_arm_target );
    m_arm->GoToWristAngle( m_wrist_target );
  }
}

// Called once the command ends or is interrupted.
void MoveMechanism::Ending(bool interrupted) {}

// Returns true when the command should end.
bool MoveMechanism::IsFinished() {
  return m_elev_is_done && m_wrist_retracted && m_arm->IsAtGoal() && m_elev->IsAtGoal();
}

bool MoveMechanism::FlipArmForward() {

  if( !m_wrist_retracted ) {
      // The wrist hasn't retracted yet.
    m_arm->GoToWristAngle( 35_deg );
    if( units::math::abs( m_arm->GetWristAngle() - 35_deg ) < 5_deg ) {
      // We got the wrist to the correct place.
      m_wrist_retracted = true;
      m_arm->GoToArmAngle( m_arm_target );
    } 
  } else {
      // The wrist is retracted but the arm still needs to move forward.
    if( m_arm->GetArmAngle() - 80_deg < 0_deg ) {
      // Wait until the arm angle is less than 80_deg to move the wrist
      // and elevator to the target position.
      m_arm->GoToWristAngle( m_wrist_target );
    }
  }

  return m_wrist_retracted && m_arm->IsAtGoal();
}

bool MoveMechanism::FlipArmBackward() {

  if( !m_wrist_retracted ) {
    m_arm->GoToArmAngle( m_arm_target );
    m_arm->GoToWristAngle( 35_deg );
    if( m_arm->GetArmAngle() - m_arm->GetWristAngle() < 20_deg ) {
        // The wrist has caught the arm.  Slow down the wrist.
      m_arm->GoToWristAngle( m_arm->GetArmAngle() - 20_deg );
    }
    if( units::math::abs( m_arm->GetWristAngle() - 35_deg ) < 5_deg ) {
      // We got the wrist to the correct place.
      m_wrist_retracted = true;
      m_arm->GoToArmAngle( m_arm_target );
    } 
  } else {
      // The wrist is retracted but the arm still needs to move backward.
    if( m_arm->GetArmAngle() - 120_deg > 0_deg ) {
      // Wait until the arm angle is greater than 120_deg to move the wrist
      // to the target position.
      m_arm->GoToWristAngle( m_wrist_target );
    }
  } 

  return m_wrist_retracted && m_arm->IsAtGoal();
}

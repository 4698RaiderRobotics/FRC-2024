// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "commands/ShootNoteTargeting.h"

#include "Constants.h"

#include "subsystems/SwerveDriveSubsystem.h"
#include "subsystems/IntakeSubsystem.h"
#include "subsystems/ShooterSubsystem.h"
#include "subsystems/ArmSubsystem.h"
#include "subsystems/VisionSubsystem.h"
#include "subsystems/ElevatorSubsystem.h"

#include <frc2/command/InstantCommand.h>
#include <frc2/command/WaitCommand.h>
#include <frc2/command/ParallelCommandGroup.h>

#include <frc/DriverStation.h>

#include "DataLogger.h"

ShootNoteTargeting::ShootNoteTargeting( SwerveDriveSubsystem* swerve, ShooterSubsystem* shooter, 
                      IntakeSubsystem* intake, ArmSubsystem* arm, ElevatorSubsystem* elev, 
                      VisionSubsystem* vision, ControllerAxis *x_axis, ControllerAxis *y_axis ) :
                      m_drive{swerve}, m_shooter{shooter}, m_intake{intake}, m_arm{arm}, m_elev{elev},
                      m_vision{vision}, m_x_axis{x_axis}, m_y_axis{y_axis}
{
  SetName( "ShootNoteTargeting" );

  if( x_axis != nullptr ) {
    allowDriving = true;
  } else {
    allowDriving = false;
  }

  AddRequirements({m_drive, m_shooter, m_intake, m_arm, m_elev, m_vision});
}

// Called when the command is initially scheduled.
void ShootNoteTargeting::Init() {
    // Start the shooter motors and move to the correct arm and wrist positions.
  // m_arm->GoToArmAngle( m_shooter->GetShooter_ArmAngle() );
      m_arm->SetWristGoal( 180_deg - m_shooter->GetAngle() );

  // fmt::print("Setting Arm / Wrist / Elevator to {}/{}/{}\n", 
  //             m_shooter->GetShooter_ArmAngle(), 180_deg - m_shooter->GetAngle(), 
  //             m_shooter->GetShooter_ElevatorHeight() );

  if(frc::DriverStation::GetAlliance().value() == frc::DriverStation::Alliance::kRed) {
        // 2D location of AprilTag #4
    targetLocation = {652.73_in, 218.42_in, 180_deg};
  } else {
        // 2D location of AprilTag #7
    targetLocation = {-1.5_in, 218.42_in, 0_deg};
  }

  frc::Pose2d robotPose = m_drive->GetPose();
  units::meter_t dist_to_speaker = (robotPose - targetLocation).Translation().Norm();
  m_shooter->SetRPMGoal( 1000_rpm + 200_rpm * dist_to_speaker.value() );

  readyToShoot = false;
  noTargets = false;
}

// Called repeatedly when this Command is scheduled to run
void ShootNoteTargeting::Execute() {
  units::degree_t arm_tolerance = 7_deg;

  if( !readyToShoot ) {
      // Set the shooter angle based on the distance between the robot and the speaker.
    frc::Pose2d robotPose = m_drive->GetPose();
    units::meter_t delta_x = robotPose.X() - targetLocation.X();
    units::meter_t delta_y =  robotPose.Y() - targetLocation.Y();

    units::meter_t dist_to_speaker = (robotPose - targetLocation).Translation().Norm();

    DataLogger::SendNT( "ShootNote/SpeakerToRobot dist", dist_to_speaker.value() );

    units::degree_t azimuthAngle;
    units::degree_t azimuthCorrAngle;
    units::degree_t shooterAngle;
    if( dist_to_speaker < 6_m  ) {
      azimuthAngle = units::math::atan2( 78_in, dist_to_speaker );
      azimuthCorrAngle = azimuthCorrection.lookup( dist_to_speaker.value() ) * 1_deg;
      shooterAngle = azimuthAngle + azimuthCorrAngle;
      m_shooter->SetRPMGoal( 1300_rpm + 225_rpm * dist_to_speaker.value() );
    } else {
        // Lob shot from more than 6_m away
      azimuthAngle = 50_deg;
      shooterAngle = 50_deg;
      m_shooter->SetRPMGoal( 1000_rpm + 100_rpm * (dist_to_speaker - 6_m ).value() );
        // Bias toward the Amp
      delta_y -= 50_in;
    }
    units::degree_t planeAngle =  units::math::atan2( delta_y, delta_x );

    m_shooter->SetAngleGoal( shooterAngle );
    if( m_arm->GetWristAngle() > 80_deg ) {
      m_arm->SetArmGoal( m_shooter->GetShooter_ArmAngle() );
      m_elev->SetGoal( m_shooter->GetShooter_ElevatorHeight() );
    }
    m_arm->SetWristGoal( 180_deg - shooterAngle );

    

      // Yaw to target in degrees
    units::degree_t t_yaw = planeAngle - robotPose.Rotation().Degrees();
    if( t_yaw > 180.0_deg ) {
      t_yaw -= 360.0_deg;
    } else if( t_yaw < -180_deg ) {
      t_yaw += 360_deg;
    }


    // fmt::print( "Deltas ({:.5},{:.5}), plane angle ({:.5}), robot angle ({:.5}), t_taw = {:.5}\n", delta_x, 
    // delta_y, planeAngle, m_drive->GetPose().Rotation().Degrees(), t_yaw );

    double turnCorrection = (t_yaw.value() + yawOffset) * 0.02;

    if( allowDriving ) {
      m_drive->ArcadeDrive( m_x_axis->GetAxis(), m_y_axis->GetAxis(), turnCorrection );
    } else {
      m_drive->ArcadeDrive( 0, 0, turnCorrection );
    }

    // DataLogger::SendNT( "ShootNote/Shooter Angle Goal", shooterAngle.value() );
    // DataLogger::SendNT( "ShootNote/Shooter Angle", m_shooter->GetAngle().value() );
    // DataLogger::SendNT( "ShootNote/Arm Angle Goal", m_shooter->GetShooter_ArmAngle().value() );
    // DataLogger::SendNT( "ShootNote/Arm Angle", m_arm->GetArmAngle().value() );
    // DataLogger::SendNT( "ShootNote/Wrist Angle Goal", 180 - shooterAngle.value() );
    // DataLogger::SendNT( "ShootNote/Wrist Angle", m_arm->GetWristAngle().value() );
    // DataLogger::SendNT( "ShootNote/Elevator Height", m_elev->GetHeight().value() );
    // DataLogger::SendNT( "ShootNote/Elevator Goal",  m_shooter->GetShooter_ElevatorHeight().value() );
    // DataLogger::SendNT( "ShootNote/Arm IsAtGoal", m_arm->IsAtGoal( arm_tolerance ) );
    // DataLogger::SendNT( "ShootNote/Shooter IsAtGoal", m_shooter->IsAtGoal() );
    // DataLogger::SendNT( "ShootNote/Elevator IsAtGoal", m_elev->IsAtGoal() );
    DataLogger::SendNT( "ShootNote/Target Yaw", t_yaw.value() );
    DataLogger::SendNT( "ShootNote/DeltaX", delta_x.value() );
    DataLogger::SendNT( "ShootNote/DeltaY", delta_y.value() );
    DataLogger::SendNT( "ShootNote/Azimuth", azimuthAngle.value() );
    DataLogger::SendNT( "ShootNote/Azimuth Correction", azimuthCorrAngle.value() );
    DataLogger::SendNT( "ShootNote/Plane Angle", planeAngle.value() );
    DataLogger::SendNT( "ShootNote/Turn Correction", turnCorrection );

    // fmt::print( "    Arm/Wrist Angles:  atGoal({}), Curr_arm({}), Arm_target({}), Curr_wrist({}), Wrist_target({})\n", 
    //                 m_arm->IsAtGoal( arm_tolerance ), m_arm->GetArmAngle(), m_shooter->GetShooter_ArmAngle(),
    //                  m_arm->GetWristAngle(), 180_deg - targetAngle );
    if( m_shooter->AtGoal() && m_arm->AtGoal( arm_tolerance ) && fabs(t_yaw.value() + yawOffset) < 1.5 ) {
      // fmt::print( "ShootNoteTargeting(), pitch({}), Shooter({}), Arm({}), Wrist({})\n", t_pitch, targetAngle,
      //              m_shooter->GetShooter_ArmAngle(), 180_deg - targetAngle  );
      readyToShoot = true;
      m_intake->SpinIntake( -1 );
      spin_start = frc::Timer::GetFPGATimestamp();
    }
    
    // else {
    //   DataLogger::Log( fmt::format( "ShootNoteTargeting -- TOO Far from target dist = {}", dist_to_speaker ));
    //   noTargets = true;
    // }
  } else {
      // We are currently shooting
    m_drive->ArcadeDrive( 0, 0, 0 );
    if( frc::Timer::GetFPGATimestamp() - spin_start > 0.5_s ) {
        // We are done shooting...
      m_intake->SpinIntake( 0.0 );
      noTargets = true;  // No ammo...
    }
  }
}

// Called once the command ends or is interrupted.
void ShootNoteTargeting::Ending(bool interrupted) {
  // fmt::print( "ShootNoteTargeting::End interrupted({}), noTargets({})\n", interrupted, noTargets );
  if( m_shooter->GetAngle() > 50_deg ) {
    // Shooter is up too far.  It will hit the intake.
    m_shooter->SetAngleGoal( 50_deg );
  }

    // Don't completely stop the shooter in auto mode.
  // if( frc::DriverStation::IsAutonomous() ) {
  //   m_shooter->SetRPMGoal( 1000_rpm );
  // } else {
    m_shooter->SetRPMGoal( 0_rpm );
  // }

  m_arm->SetArmGoal( 170_deg );
  m_arm->SetWristGoal( 35_deg );
  m_elev->SetGoal(0_in);
}

// Returns true when the command should end.
bool ShootNoteTargeting::IsFinished() {

  return noTargets;
}

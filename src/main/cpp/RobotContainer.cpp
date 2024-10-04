// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "RobotContainer.h"

#include <frc2/command/Commands.h>
#include <frc2/command/RunCommand.h>
#include <frc2/command/SequentialCommandGroup.h>
#include <frc2/command/button/JoystickButton.h>
#include <frc2/command/RepeatCommand.h>
#include <frc2/command/ParallelCommandGroup.h>
#include <frc2/command/ConditionalCommand.h>
#include <frc2/command/FunctionalCommand.h>
#include <frc2/command/WaitCommand.h>

#include <frc/smartdashboard/SmartDashboard.h>

#include <pathplanner/lib/auto/AutoBuilder.h>
#include <pathplanner/lib/auto/NamedCommands.h>

#include "commands/ChangeShooterAngle.h"
#include "commands/ChangeArmAngle.h"
#include "commands/ChangeWristAngle.h"
#include "commands/IntakeNote.h"
#include "commands/PickUpNote.h"
#include "commands/ShootNoteTargeting.h"
#include "commands/StageNoteInShooter.h"
#include "commands/GotoRestPosition.h"
#include "commands/ProfiledDriveToPose.h"
#include "commands/ChangeClimberHeight.h"
#include "commands/Climb.h"
#include "commands/ClimbAndTrap.h"
#include "commands/ChangeElevatorHeight.h"
#include "commands/MoveMechanism.h"
#include "commands/MoveToAndPlaceInAmp.h"
#include "commands/AutoClimbAndTrap.h"

#include "Constants.h"

RobotContainer::RobotContainer() 
: m_swerveDrive{&m_vision}, m_intake{&m_leds} {

  m_swerveDrive.SetDefaultCommand(frc2::RunCommand(
    [this] {
      m_swerveDrive.ArcadeDrive(vx_axis.GetAxis(), vy_axis.GetAxis(), omega_axis.GetAxis());
   },
    { &m_swerveDrive }
    ).WithName("Arcade Drive"));

  m_arm.SetDefaultCommand(frc2::RunCommand(
    [this] {
      if(m_operatorController.GetLeftBumper()) {
        m_arm.NudgeArmAngle(arm_angle_axis.GetAxis() * 0.5_deg);
        m_arm.NudgeWristAngle(wrist_angle_axis.GetAxis() * 0.5_deg);
      }
    },
    { &m_arm }
    ).WithName("Arm Nudge"));

  m_elevator.SetDefaultCommand(frc2::RunCommand(
    [this] {
      if(m_operatorController.GetLeftBumper()) {
        m_elevator.NudgeHeight(elevator_axis.GetAxis() * 0.1_in);
      }
    },
    { &m_elevator }
    ).WithName("Elevator Nudge"));

  m_shooter.SetDefaultCommand(frc2::RunCommand(
    [this] {
      if(m_operatorController.GetLeftBumper()) {
        if( m_operatorController.GetPOV() == 90 ) {
          m_shooter.Nudge( 0.1_deg ); 
        } else if( m_operatorController.GetPOV() == 270 ) {
          m_shooter.Nudge( -0.1_deg );
        }
      }
    },
    { &m_shooter }
    ).WithName("Shooter Nudge"));

  m_climber.SetDefaultCommand(frc2::RunCommand(
    [this] {

      if( m_operatorController.GetPOV() == 0 ) {
        m_climber.SetSpeed( 0.5 ); 
      } else if( m_operatorController.GetPOV() == 180 ) {
        m_climber.SetSpeed( -0.5 );
      } else {
        m_climber.SetSpeed( 0.0 );
      }
    },
    { &m_climber }
    ).WithName("Climber Nudge"));


  ConfigureBindings();

  ConfigureAutos();

  frc::SmartDashboard::PutData("Auto Mode", &m_chooser);
}

void RobotContainer::ConfigureBindings() {

  //    **********************  DRIVER CONTROLS *********************

    // Resets the gyro when the robot is facing away from the driver
  (m_driverController.L1() && m_driverController.R1() )
    .OnTrue(frc2::InstantCommand([this] { m_swerveDrive.ResetDriverOrientation(0_deg); }, { &m_swerveDrive }).ToPtr());

    // Eject Note into the amp.
  m_driverController.L2().OnTrue(frc2::SequentialCommandGroup(
    frc2::SequentialCommandGroup(ChangeArmAngle(&m_arm, physical::kArmAmpAngle), ChangeWristAngle(&m_arm, physical::kWristAmpSpitAngle)),
    frc2::InstantCommand([this] {m_intake.SpinIntake(0.5);}, {&m_intake}),
    frc2::WaitCommand(0.5_s),
    frc2::InstantCommand([this] {m_intake.SpinIntake(0.0);}, {&m_intake}),
    frc2::SequentialCommandGroup(ChangeArmAngle(&m_arm, physical::kArmAmpDropAngle), ChangeWristAngle(&m_arm, physical::kWristAmpDropAngle)),
    ChangeElevatorHeight(&m_elevator, 0_m),
    GoToRestPosition( &m_arm, &m_elevator, &m_intake )).ToPtr().WithName( "Driver Put in Amp")
  );

    // Automatically move to the amp and place piece
  m_driverController.R2().OnTrue(
    frc2::InstantCommand( 
      [this] { 
        delete m_moveToAmpCmd;
        m_moveToAmpCmd = new MoveToAndPlaceInAmp(&m_swerveDrive, &m_intake, &m_arm, &m_elevator, &m_vision);
        m_moveToAmpCmd->Schedule();
      }, {}
    ).ToPtr() 
  );

    // Manual eject of the Note
  m_driverController.Cross()
    .OnTrue( frc2::InstantCommand([this] {m_intake.SpinIntake(-0.5);}, {&m_intake}).ToPtr().WithName("Driver X - Note Eject"))
    .OnFalse(frc2::InstantCommand([this] {m_intake.SpinIntake(0.0);}, {&m_intake}).ToPtr().WithName("Driver X - Note Eject Stop"));




  //    **********************  OPERATOR CONTROLS *********************

    // Pickup Note
  m_operatorController.A().OnTrue(PickUpNote(&m_intake, &m_arm, &m_elevator).ToPtr());

    // Goto Rest Position
  m_operatorController.B().OnTrue(
      frc2::SequentialCommandGroup(
        ChangeElevatorHeight(&m_elevator, 0_in), 
        frc2::ParallelCommandGroup(
          GoToRestPosition( &m_arm, &m_elevator, &m_intake ),
          frc2::InstantCommand([this] {m_intake.SpinIntake(0.0); m_shooter.Spin(0_rpm);}, {&m_intake, &m_shooter})
        )
      ).ToPtr().WithName( "B Button - Rest Position" )
    );

    // Prepare to Place in Amp
  m_operatorController.X().OnTrue(
    frc2::SequentialCommandGroup(
      frc2::SequentialCommandGroup(
        ChangeArmAngle(&m_arm, physical::kArmAmpAngle), 
        ChangeWristAngle(&m_arm, physical::kWristAmpAngle)
      ),
      ChangeElevatorHeight(&m_elevator, physical::kElevatorAmpHeight)
    ).ToPtr().WithName("Button X -- Prepare for Amp")
  );

    // Prepare to Climb
  m_operatorController.Y().OnTrue(
    frc2::SequentialCommandGroup(
      frc2::ParallelCommandGroup(
        frc2::SequentialCommandGroup(
          ChangeArmAngle(&m_arm, physical::kArmAmpAngle), 
          ChangeWristAngle(&m_arm, physical::kWristAmpAngle)
        ),
        ChangeShooterAngle(&m_shooter, 60_deg)
      ), 
      ChangeElevatorHeight(&m_elevator, physical::kElevatorTrapHeight)
    ).ToPtr().WithName( "Button Y - Pre Climb")
  );

    // Shoot Note
  (m_operatorController.RightBumper() && !m_operatorController.LeftStick()).OnTrue(
    frc2::ConditionalCommand(
      frc2::SequentialCommandGroup(
        frc2::InstantCommand( [this] { m_shooter.Spin( 1000_rpm ); m_swerveDrive.ArcadeDrive( 0, 0, 0 ); }, {&m_swerveDrive, &m_shooter} ),
        GoToRestPosition( &m_arm, &m_elevator, &m_intake ),
        ShootNoteTargeting( &m_swerveDrive, &m_shooter, &m_intake, &m_arm, &m_elevator, &m_vision, &vx_axis, &vy_axis )),
      frc2::InstantCommand(), 
      [this] {return m_intake.HasNote();}
    ).ToPtr().WithName( "Right Bumper  - ShootNoteTargeting")
  );

    // Auto Climb and Trap
  m_operatorController.LeftTrigger().OnTrue( 
    frc2::InstantCommand( 
      [this] { 
        delete m_climbAndTrapCmd;
        m_climbAndTrapCmd = new AutoClimbAndTrap(&m_swerveDrive, &m_intake, &m_arm, &m_elevator, &m_climber, &m_shooter, &m_vision);
        m_climbAndTrapCmd->Schedule();
      }, {}
    ).ToPtr() 
  );

    // Climb and Trap after Pre-setup with Button Y
  m_operatorController.RightTrigger().OnTrue(ClimbAndTrap(&m_shooter, &m_intake, &m_climber, &m_arm, &m_elevator).ToPtr());

    // Stage Note in Shooter for testing purposes
 (m_operatorController.RightBumper() && m_operatorController.LeftStick()).OnTrue(
      frc2::SequentialCommandGroup( 
        GoToRestPosition( &m_arm, &m_elevator, &m_intake ),
        StageNoteInShooter( &m_shooter, &m_intake, &m_arm, &m_elevator )
      ).ToPtr().WithName( "Right Bumper + Left Stick - StageNoteInShooter")
    );
}


void RobotContainer::ConfigureAutos() {

 pathplanner::NamedCommands::registerCommand("pickUpNote", PickUpNote(&m_intake, &m_arm, &m_elevator).ToPtr());
  pathplanner::NamedCommands::registerCommand("shootNoteTargeting", 
    frc2::SequentialCommandGroup(
      MoveMechanism( &m_arm, &m_elevator, physical::kArmPassiveAngle, 130_deg, 0_in ), 
      ShootNoteTargeting(&m_swerveDrive, &m_shooter, &m_intake, &m_arm, &m_elevator, &m_vision )
    ).ToPtr()
  );

  std::vector<AutoNameMap> autos = {
    { "One Piece", "THEOnePiece" },
    { "Source Four Piece", "SourceFourPiece" },
    { "Source Three Piece", "SourceThreePiece" },
    { "Source Two Piece", "SourceTwoPiece" },
    { "Source Two Piece Center", "SourceTwoPieceCenter" },
    { "Source Three Piece Center", "SourceThreePieceCenter" },
    { "Source Three Piece Center Wait", "SourceThreePieceCenterWait" },
    { "Source Four Piece Center", "SourceFourPieceCenter" },
    { "Source One Piece Taxi", "SourceOnePieceTaxi" },

    { "Amp Four Piece", "AmpFourPiece" },
    { "Amp Three Piece", "AmpThreePiece" },
    { "Amp Two Piece", "AmpTwoPiece" },
    { "Amp Three Piece Center", "AmpThreePieceCenter" },
    { "Amp Four Piece Center", "AmpFourPieceCenter" },

    { "Middle Four Piece", "MiddleFourPiece" },
    { "Middle Three Piece Amp", "MiddleThreePieceAmp" },
    { "Middle Three Piece Source", "MiddleThreePieceSource" },
    { "Middle Two Piece", "MiddleTwoPiece" },
    { "Middle Four Piece Center Amp", "MiddleFourPieceCenterAmp" },
    { "Middle Four Piece Center Source", "MiddleFourPieceCenterSource" },
    { "Middle Five Piece Amp", "MiddleFivePieceAmp" }
  };

  for( unsigned int i=0; i<autos.size(); ++i ) {
      m_chooser.AddOption( autos[i].Description, i );
       AutoCommands.push_back( pathplanner::AutoBuilder::buildAuto( autos[i].AutoName ).WithName(autos[i].AutoName) );
  }
  m_chooser.SetDefaultOption( autos[0].Description, 0 );

}

frc2::Command* RobotContainer::GetAutonomousCommand() {
  return AutoCommands[ m_chooser.GetSelected() ].get();
}

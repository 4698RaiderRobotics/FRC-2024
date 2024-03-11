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

#include <frc/smartdashboard/SmartDashboard.h>

#include "commands/SpinShooter.h"
#include "commands/ChangeShooterAngle.h"
#include "commands/ChangeArmAngle.h"
#include "commands/ChangeWristAngle.h"
#include "commands/IntakeNote.h"
#include "commands/PickUpNote.h"
#include "commands/ShootNote.h"
#include "commands/ShootNoteTargeting.h"
#include "commands/PlaceInAmp.h"
#include "commands/ProfiledDriveToPose.h"
#include "commands/ChangeClimberHeight.h"
#include "commands/Climb.h"
#include "commands/ClimbAndTrap.h"
#include "commands/ChangeElevatorHeight.h"
#include "commands/autonomous/FollowTrajectory.h"

#include "commands/autonomous/TwoPieceMiddleAuto.h"
#include "commands/autonomous/OnePieceAuto.h"
#include "commands/autonomous/TwoPieceSideAuto.h"
#include "commands/autonomous/OnePieceTaxiAuto.h"

RobotContainer::RobotContainer() 
: m_swerveDrive{&m_vision}, m_intake{&m_leds} {
  m_swerveDrive.SetDefaultCommand(frc2::RunCommand(
    [this] {
      m_swerveDrive.ArcadeDrive(vx_axis.GetAxis(), vy_axis.GetAxis(), omega_axis.GetAxis());
   },
    { &m_swerveDrive }
    ));

  m_arm.SetDefaultCommand(frc2::RunCommand(
    [this] {
      if(m_operatorController.GetLeftBumper()) {
        m_arm.NudgeArmAngle(arm_angle_axis.GetAxis() * 0.5_deg);
        m_arm.NudgeWristAngle(wrist_angle_axis.GetAxis() * 0.5_deg);
      }
    },
    { &m_arm }
    ));

  m_elevator.SetDefaultCommand(frc2::RunCommand(
    [this] {
      if(m_operatorController.GetLeftBumper()) {
        m_elevator.NudgeHeight(elevator_axis.GetAxis() * 0.5_in);
      }
    },
    { &m_elevator }
    ));

  m_shooter.SetDefaultCommand(frc2::RunCommand(
    [this] {
      if(m_operatorController.GetLeftBumper()) {
        if( m_operatorController.GetPOV() == 90 ) {
          m_shooter.Nudge( 0.5_deg ); 
        } else if( m_operatorController.GetPOV() == 270 ) {
          m_shooter.Nudge( -0.5_deg );
        }
      }
    },
    { &m_shooter }
    ));

  m_climber.SetDefaultCommand(frc2::RunCommand(
    [this] {

      if( m_operatorController.GetPOV() == 0 ) {
        m_climber.SetSpeed( 0.75 ); 
      } else if( m_operatorController.GetPOV() == 180 ) {
        m_climber.SetSpeed( -0.75 );
      } else {
        m_climber.SetSpeed( 0.0 );
      }
    },
    { &m_climber }
    ));


  ConfigureBindings();

  m_chooser.SetDefaultOption(kTwoPieceMiddle, kTwoPieceMiddle);
  m_chooser.AddOption(kOnePiece, kOnePiece);
  m_chooser.AddOption(kTwoPieceLeft, kTwoPieceLeft);
  m_chooser.AddOption(kTwoPieceRight, kTwoPieceRight);
  m_chooser.AddOption(kOnePieceTaxi, kOnePieceTaxi);
  frc::SmartDashboard::PutData("Auto Modes", &m_chooser);
}

void RobotContainer::ConfigureBindings() {
  // Resets the gyro when the robot is facing the driver
  (frc2::JoystickButton(&m_driverController, frc::PS5Controller::Button::kL1) && frc2::JoystickButton(&m_driverController, frc::PS5Controller::Button::kR1))
    .OnTrue(frc2::InstantCommand([this] { m_swerveDrive.ResetGyro(0_deg); }, { &m_swerveDrive }).ToPtr());


  // m_operatorController.A().OnTrue(SpinShooter(&m_shooter, 1700_rpm).ToPtr()).OnFalse(SpinShooter(&m_shooter, 0_rpm).ToPtr());

  // m_operatorController.B().OnTrue(ChangeShooterAngle(&m_shooter, 32_deg).ToPtr());

  // m_operatorController.X().OnTrue(ChangeShooterAngle(&m_shooter, 45_deg).ToPtr());

  m_operatorController.RightBumper().OnTrue(frc2::SequentialCommandGroup(ChangeElevatorHeight(&m_elevator, 0_in),
                                                                         frc2::SequentialCommandGroup(ChangeArmAngle(&m_arm, 170_deg), 
                                                                                                      ChangeWristAngle(&m_arm, 35_deg)),  
                                                                         ShootNoteTargeting(&m_swerveDrive, &m_shooter, &m_intake, &m_arm, &m_vision, &vx_axis, &vy_axis )).ToPtr());

  // m_operatorController.RightStick().OnTrue(frc2::SequentialCommandGroup(ChangeArmAngle(&m_arm, 170_deg), ChangeWristAngle(&m_arm, 140_deg)).ToPtr());
  
  // m_operatorController.LeftStick().OnTrue(IntakeNote(&m_intake).ToPtr());

  m_operatorController.B().OnTrue(frc2::SequentialCommandGroup(ChangeElevatorHeight(&m_elevator, 0_in), 
                                                               frc2::ParallelCommandGroup(frc2::SequentialCommandGroup(ChangeArmAngle(&m_arm, 170_deg), 
                                                                                                                       ChangeWristAngle(&m_arm, 35_deg)), 
                                                                                          frc2::InstantCommand([this] {m_intake.SpinIntake(0.0);}, {&m_intake}))).ToPtr());

  m_operatorController.A().OnTrue(PickUpNote(&m_swerveDrive, &m_intake, &m_arm, &m_elevator).ToPtr());

  // m_operatorController.Y().OnTrue(ShootNote(&m_swerveDrive, &m_shooter, &m_intake, &m_arm, 25_deg, 145_deg, 150_deg).ToPtr());

  // m_operatorController.B().OnTrue(ShootNote(&m_swerveDrive, &m_shooter, &m_intake, &m_arm, 45_deg, 180_deg, 130_deg).ToPtr());

  m_operatorController.X().OnTrue(PlaceInAmp(&m_swerveDrive, &m_elevator, &m_intake, &m_arm).ToPtr());

  // m_operatorController.B().OnTrue(Climb(&m_climber).ToPtr());

  m_operatorController.RightTrigger().OnTrue(ClimbAndTrap(&m_shooter, &m_intake, &m_climber, &m_arm, &m_elevator).ToPtr());

  // Pre-climb button
  m_operatorController.Y().OnTrue(frc2::SequentialCommandGroup(frc2::ParallelCommandGroup(frc2::SequentialCommandGroup(ChangeArmAngle(&m_arm, 75_deg), 
                                                                                                                                   ChangeWristAngle(&m_arm, 90_deg)),
                                                                                                      ChangeShooterAngle(&m_shooter, 30_deg)), 
                                                                          ChangeElevatorHeight(&m_elevator, 25_in)).ToPtr());

  // m_operatorController.RightTrigger().OnTrue(FollowTrajectory(&m_swerveDrive, m_swerveDrive.exampleTraj, 0_deg).ToPtr());

  // m_operatorController.LeftTrigger().OnTrue(ChangeClimberHeight(&m_climber, 300).ToPtr());

  // m_operatorController.RightTrigger().OnTrue(ChangeClimberHeight(&m_climber, 10).ToPtr());

   frc2::JoystickButton(&m_driverController, frc::PS5Controller::Button::kCross).OnTrue(frc2::InstantCommand([this] {m_intake.SpinIntake(-1);}, {&m_intake}).ToPtr())
                          .OnFalse(frc2::InstantCommand([this] {m_intake.SpinIntake(0.0);}, {&m_intake}).ToPtr());

  // frc2::JoystickButton(&m_driverController, frc::PS5Controller::Button::kCircle).WhileTrue(frc2::RunCommand([this] {m_elevator.NudgeHeight(0.1_in);}, {&m_elevator}).ToPtr());

  // frc2::JoystickButton(&m_driverController, frc::PS5Controller::Button::kCross).WhileTrue(frc2::RunCommand([this] {m_elevator.NudgeHeight(-0.1_in);}, {&m_elevator}).ToPtr());

  // frc2::JoystickButton(&m_driverController, frc::PS5Controller::Button::kSquare).OnTrue(PlaceInAmp(&m_swerveDrive, &m_elevator, &m_intake, &m_arm).ToPtr());

  // frc2::JoystickButton(&m_driverController, frc::PS5Controller::Button::kTriangle).OnTrue(frc2::InstantCommand([this] {m_intake.SpinIntake(0.5);}, {&m_intake}).ToPtr())
  //                                                                                 .OnFalse(frc2::InstantCommand([this] {m_intake.SpinIntake(0.0);}, {&m_intake}).ToPtr());
}

frc2::Command* RobotContainer::GetAutonomousCommand() {
  delete m_autoCommand;
  m_autoCommand = nullptr;

  m_autoSelected = m_chooser.GetSelected();

  if (m_autoSelected == kTwoPieceMiddle) {
    m_autoCommand = new TwoPieceMiddleAuto(&m_swerveDrive, &m_shooter, &m_intake, &m_arm, &m_elevator, &m_vision);
  } else if (m_autoSelected == kOnePiece) {
    m_autoCommand = new OnePieceAuto(&m_swerveDrive, &m_shooter, &m_intake, &m_arm, &m_vision);
  } else if (m_autoSelected == kTwoPieceLeft) {
    m_autoCommand = new TwoPieceSideAuto(&m_swerveDrive, &m_shooter, &m_intake, &m_arm, &m_elevator, &m_vision, true);
  } else if (m_autoSelected == kTwoPieceRight) {
    m_autoCommand = new TwoPieceSideAuto(&m_swerveDrive, &m_shooter, &m_intake, &m_arm, &m_elevator, &m_vision, false);
  } else if (m_autoSelected == kOnePieceTaxi) {
    m_autoCommand = new OnePieceTaxiAuto(&m_swerveDrive, &m_shooter, &m_intake, &m_arm, &m_elevator, &m_vision);
  }

  return m_autoCommand;
}

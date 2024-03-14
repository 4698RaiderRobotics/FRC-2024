// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "commands/ShootNote.h"

#include "Constants.h"

#include "commands/ChangeShooterAngle.h"
#include "commands/SpinShooter.h"
#include "commands/ChangeArmAngle.h"
#include "commands/ChangeWristAngle.h"

#include <frc2/command/InstantCommand.h>
#include <frc2/command/WaitCommand.h>
#include <frc2/command/ParallelCommandGroup.h>


// NOTE:  Consider using this command inline, rather than writing a subclass.
// For more information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
ShootNote::ShootNote(SwerveDriveSubsystem* swerve, ShooterSubsystem* shooter, IntakeSubsystem* intake, ArmSubsystem* arm, 
                     units::degree_t shooterAngle, units::degree_t armAngle, units::degree_t wristAngle) {
  // Add your commands here, e.g.
  // AddCommands(FooCommand{}, BarCommand{});

  SetName( "ShootNote" );

  AddCommands(
    // AprilTag data
    // Drive to location + spin
    frc2::ParallelCommandGroup(SpinShooter(shooter, 2000_rpm),
                               frc2::SequentialCommandGroup(ChangeArmAngle(arm, armAngle), ChangeWristAngle(arm, wristAngle))),
    ChangeShooterAngle(shooter, shooterAngle), 
    frc2::InstantCommand([this, intake] {intake->SpinIntake(-1);}, {intake}),
    frc2::WaitCommand(0.5_s),
    frc2::InstantCommand([this, intake] {intake->SpinIntake(0.0);}, {intake}),
    SpinShooter(shooter, 0_rpm),
    ChangeShooterAngle(shooter, 30_deg),
    frc2::SequentialCommandGroup(ChangeArmAngle(arm, 170_deg), ChangeWristAngle(arm, 35_deg))
  );
}

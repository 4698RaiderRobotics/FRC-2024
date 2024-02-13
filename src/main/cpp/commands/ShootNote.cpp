// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "commands/ShootNote.h"

#include "Constants.h"

#include "commands/ChangeShooterAngle.h"
#include "commands/SpinShooter.h"
#include "commands/ChangeArmAngle.h"

#include <frc2/command/InstantCommand.h>
#include <frc2/command/WaitCommand.h>


// NOTE:  Consider using this command inline, rather than writing a subclass.
// For more information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
ShootNote::ShootNote(SwerveDriveSubsystem* swerve, ShooterSubsystem* shooter, IntakeSubsystem* intake, ArmSubsystem* arm, units::degree_t shooterAngle) {
  // Add your commands here, e.g.
  // AddCommands(FooCommand{}, BarCommand{});
  AddCommands(
    // AprilTag data
    // Drive to location + spin
    ChangeShooterAngle(shooter, shooterAngle),
    ChangeArmAngle(arm, physical::kArmShootingAngle, physical::kWristShootingAngle),
    SpinShooter(shooter, physical::kShooterSpeed),
    frc2::InstantCommand([this, intake] {intake->SpinIntake(0.5);}, {intake}),
    frc2::WaitCommand(0.1_s),
    frc2::InstantCommand([this, intake] {intake->SpinIntake(0.0);}, {intake}),
    ChangeArmAngle(arm, physical::kArmPassiveAngle, physical::kWristPassiveAngle)
  );
}

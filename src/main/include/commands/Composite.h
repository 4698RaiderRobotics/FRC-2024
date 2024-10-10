#pragma once

#include <frc2/command/CommandPtr.h>
#include <frc2/command/CommandScheduler.h>

class SwerveDriveSubsystem;
class IntakeSubsystem;
class ArmSubsystem;
class ElevatorSubsystem;
class ClimberSubsystem;
class ShooterSubsystem;
class VisionSubsystem;


frc2::CommandPtr PickUpNote( IntakeSubsystem* intake, ArmSubsystem* arm, ElevatorSubsystem *elevator);

frc2::CommandPtr DriverPlaceInAmp( ArmSubsystem* arm, ElevatorSubsystem *elevator, IntakeSubsystem* intake );

frc2::CommandPtr ClimbAndTrap(ShooterSubsystem* shooter, IntakeSubsystem* intake, ClimberSubsystem *climber, 
                      ArmSubsystem* arm, ElevatorSubsystem *elevator);


class MoveToAndPlaceInAmp {
public:
    MoveToAndPlaceInAmp( SwerveDriveSubsystem* drive, IntakeSubsystem* intake, ArmSubsystem* arm,
                         ElevatorSubsystem *elevator, VisionSubsystem* vision);

    void Schedule();

private:
    SwerveDriveSubsystem* drive;
    IntakeSubsystem* intake;
    ArmSubsystem* arm;
    ElevatorSubsystem *elevator;
    VisionSubsystem* vision;

    frc::Pose2d amp_tags[2];

    std::vector<frc2::CommandPtr> commands;
};

class AutoClimbAndTrap {
public:
    AutoClimbAndTrap( SwerveDriveSubsystem* drive, IntakeSubsystem* intake, ArmSubsystem* arm,
                      ElevatorSubsystem *elevator, ClimberSubsystem *climb, ShooterSubsystem* shooter, VisionSubsystem* vision);

    void Schedule();

private:
    SwerveDriveSubsystem* drive;
    IntakeSubsystem* intake;
    ArmSubsystem* arm;
    ElevatorSubsystem *elevator;
    ClimberSubsystem *climb;
    ShooterSubsystem* shooter;
    VisionSubsystem* vision;

    frc::Pose2d stage_tags[6];

        // Map the April Tag number to a command.
    std::vector<frc2::CommandPtr> commands;
};

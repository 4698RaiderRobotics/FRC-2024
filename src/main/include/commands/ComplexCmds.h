#pragma once

#include <frc2/command/CommandPtr.h>

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


class ConstructOnDemandCommand {
public:
    ConstructOnDemandCommand() = default;
    
    virtual void Schedule() final { 
        cmd = CommandFactory().Unwrap();
        cmd->Schedule();
    } 
    
private:
    virtual frc2::CommandPtr CommandFactory() = 0;

    std::unique_ptr<frc2::Command> cmd;
};

class MoveToAndPlaceInAmp : public ConstructOnDemandCommand {
public:
    MoveToAndPlaceInAmp( SwerveDriveSubsystem* drive, IntakeSubsystem* intake, ArmSubsystem* arm,
                         ElevatorSubsystem *elevator, VisionSubsystem* vision) 
                         : drive{drive}, intake{intake}, arm{arm}, elevator{elevator}, vision{vision} {}

private:
    frc2::CommandPtr CommandFactory();

    SwerveDriveSubsystem* drive;
    IntakeSubsystem* intake;
    ArmSubsystem* arm;
    ElevatorSubsystem *elevator;
    VisionSubsystem* vision;
};

class AutoClimbAndTrap : public ConstructOnDemandCommand {
public:
    AutoClimbAndTrap( SwerveDriveSubsystem* drive, IntakeSubsystem* intake, ArmSubsystem* arm,
                         ElevatorSubsystem *elevator, ClimberSubsystem *climb, ShooterSubsystem* shooter, VisionSubsystem* vision) 
                         : drive{drive}, intake{intake}, arm{arm}, elevator{elevator}, climb{climb}, shooter{shooter}, vision{vision} {}
private:
    frc2::CommandPtr CommandFactory();

    SwerveDriveSubsystem* drive;
    IntakeSubsystem* intake;
    ArmSubsystem* arm;
    ElevatorSubsystem *elevator;
    ClimberSubsystem *climb;
    ShooterSubsystem* shooter;
    VisionSubsystem* vision;
};

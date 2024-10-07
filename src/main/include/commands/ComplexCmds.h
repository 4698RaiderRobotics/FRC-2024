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

frc2::CommandPtr MoveToAndPlaceInAmpImpl(SwerveDriveSubsystem* drive, IntakeSubsystem* intake, ArmSubsystem* arm, ElevatorSubsystem *elevator,
                                        VisionSubsystem* vision);

frc2::CommandPtr ClimbAndTrap(ShooterSubsystem* shooter, IntakeSubsystem* intake, ClimberSubsystem *climber, 
                      ArmSubsystem* arm, ElevatorSubsystem *elevator);

frc2::CommandPtr AutoClimbAndTrapImpl( SwerveDriveSubsystem* drive, IntakeSubsystem* intake, ArmSubsystem* arm, ElevatorSubsystem *elevator,
                                   ClimberSubsystem *climb, ShooterSubsystem* shooter, VisionSubsystem *vision );

class Stateful {
public:
    Stateful() {}
    
    void Schedule() { 
        cmd = CommandFactory().Unwrap();
        cmd->Schedule();
    }
    
private:
    virtual frc2::CommandPtr CommandFactory() = 0;

    std::unique_ptr<frc2::Command> cmd;
};

class MoveToAndPlaceInAmp : public Stateful {
public:
    MoveToAndPlaceInAmp( SwerveDriveSubsystem* drive, IntakeSubsystem* intake, ArmSubsystem* arm,
                         ElevatorSubsystem *elevator, VisionSubsystem* vision) 
                         : d{drive}, i{intake}, a{arm}, e{elevator}, v{vision} {}
private:
    frc2::CommandPtr CommandFactory() { return MoveToAndPlaceInAmpImpl(d,i,a,e,v);}

    SwerveDriveSubsystem* d;
    IntakeSubsystem* i;
    ArmSubsystem* a;
    ElevatorSubsystem *e;
    VisionSubsystem* v;
};

class AutoClimbAndTrap : public Stateful {
public:
    AutoClimbAndTrap( SwerveDriveSubsystem* drive, IntakeSubsystem* intake, ArmSubsystem* arm,
                         ElevatorSubsystem *elevator, ClimberSubsystem *climb, ShooterSubsystem* shooter, VisionSubsystem* vision) 
                         : d{drive}, i{intake}, a{arm}, e{elevator}, c{climb}, s{shooter}, v{vision} {}
private:
    frc2::CommandPtr CommandFactory() { return AutoClimbAndTrapImpl(d,i,a,e,c,s,v);}

    SwerveDriveSubsystem* d;
    IntakeSubsystem* i;
    ArmSubsystem* a;
    ElevatorSubsystem *e;
    ClimberSubsystem *c;
    ShooterSubsystem* s;
    VisionSubsystem* v;
};

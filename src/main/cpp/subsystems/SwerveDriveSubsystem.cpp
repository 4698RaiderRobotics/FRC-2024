// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "subsystems/SwerveDriveSubsystem.h"
#include "SwerveConstants.h"
#include "DataLogger.h"

#include <frc/DriverStation.h>
#include <frc/Filesystem.h>
#include <frc/trajectory/TrajectoryUtil.h>
#include <wpi/fs.h>

#include <frc/smartdashboard/SmartDashboard.h>

#include <pathplanner/lib/auto/AutoBuilder.h>
#include <pathplanner/lib/util/HolonomicPathFollowerConfig.h>
#include <pathplanner/lib/util/PIDConstants.h>
#include <pathplanner/lib/util/ReplanningConfig.h>

SwerveDriveSubsystem::SwerveDriveSubsystem(VisionSubsystem *ll) 
    : m_vision{ll}
    , m_modules{ SwerveModule{ swerve::deviceIDs::kFrontLeftTurnMotorID, swerve::deviceIDs::kFrontLeftDriveMotorID, 
                               swerve::deviceIDs::kFrontLeftAbsoluteEncoderID, swerve::physical::kFrontLeftAbsoluteOffset },
                 SwerveModule{ swerve::deviceIDs::kFrontRightTurnMotorID, swerve::deviceIDs::kFrontRightDriveMotorID, 
                               swerve::deviceIDs::kFrontRightAbsoluteEncoderID, swerve::physical::kFrontRightAbsoluteOffset },
                 SwerveModule{ swerve::deviceIDs::kBackLeftTurnMotorID, swerve::deviceIDs::kBackLeftDriveMotorID, 
                               swerve::deviceIDs::kBackLeftAbsoluteEncoderID, swerve::physical::kBackLeftAbsoluteOffset },
                 SwerveModule{ swerve::deviceIDs::kBackRightTurnMotorID, swerve::deviceIDs::kBackRightDriveMotorID, 
                               swerve::deviceIDs::kBackRightAbsoluteEncoderID, swerve::physical::kBackRightAbsoluteOffset } }
    , m_gyro{swerve::deviceIDs::kPigeonIMUID, "Drivetrain"} 
    , m_kinematics{ frc::Translation2d{+( swerve::physical::kDriveBaseLength / 2 ), +( swerve::physical::kDriveBaseWidth / 2 )},
                    frc::Translation2d{+( swerve::physical::kDriveBaseLength / 2 ), -( swerve::physical::kDriveBaseWidth / 2 )},
                    frc::Translation2d{-( swerve::physical::kDriveBaseLength / 2 ), +( swerve::physical::kDriveBaseWidth / 2 )},
                    frc::Translation2d{-( swerve::physical::kDriveBaseLength / 2 ), -( swerve::physical::kDriveBaseWidth / 2 )} }
    , m_odometry{ m_kinematics, frc::Rotation2d{ 0_deg },
                    { m_modules[0].GetPosition(), 
                      m_modules[1].GetPosition(),
                      m_modules[2].GetPosition(), 
                      m_modules[3].GetPosition()
                    }, frc::Pose2d{ 0_ft, 0_ft, 0_deg } }
    , m_controller{ frc::PIDController{ swerve::pidf::X_Holo_kP, swerve::pidf::X_Holo_kI, swerve::pidf::X_Holo_kD }, 
                    frc::PIDController{ swerve::pidf::Y_Holo_kP, swerve::pidf::Y_Holo_kI, swerve::pidf::Y_Holo_kD },
                    frc::ProfiledPIDController<units::radian> {
                        swerve::pidf::Th_Holo_kP, swerve::pidf::Th_Holo_kI, swerve::pidf::Th_Holo_kD, 
                        frc::TrapezoidProfile<units::radian>::Constraints{
                            swerve::pidf::Th_Holo_MaxVel, swerve::pidf::Th_Holo_MaxAcc }}}

{
#ifdef TUNING
        // Holonomic Controller parameters
    frc::SmartDashboard::PutNumber("X_Holo P", m_controller.getXController().GetP() );
    frc::SmartDashboard::PutNumber("X_Holo I", m_controller.getXController().GetI() );
    frc::SmartDashboard::PutNumber("X_Holo D", m_controller.getXController().GetD() );
    frc::SmartDashboard::PutNumber("Y_Holo P", m_controller.getYController().GetP() );
    frc::SmartDashboard::PutNumber("Y_Holo I", m_controller.getYController().GetI() );
    frc::SmartDashboard::PutNumber("Y_Holo D", m_controller.getYController().GetD() );
    frc::SmartDashboard::PutNumber("Th_Holo P", m_controller.getThetaController().GetP() );
    frc::SmartDashboard::PutNumber("Th_Holo I", m_controller.getThetaController().GetI() );
    frc::SmartDashboard::PutNumber("Th_Holo D", m_controller.getThetaController().GetD() );
    frc::SmartDashboard::PutNumber("Th_Holo MaxVel", swerve::pidf::Th_Holo_MaxVel.value() );
    frc::SmartDashboard::PutNumber("Th_Holo MaxAcc", swerve::pidf::Th_Holo_MaxAcc.value() );
    
        // Swerve Module parameters
    // frc::SmartDashboard::PutNumber("Drive P", m_modules[0].m_drivePIDController.GetP() );
    // frc::SmartDashboard::PutNumber("Drive I", m_modules[0].m_drivePIDController.GetI() );
    // frc::SmartDashboard::PutNumber("Drive D", m_modules[0].m_drivePIDController.GetD() );
    // frc::SmartDashboard::PutNumber("Drive FF", m_modules[0].m_drivePIDController.GetFF() );
    frc::SmartDashboard::PutNumber("Turn P", m_modules[0].m_turnPIDController.GetP() );
    frc::SmartDashboard::PutNumber("Turn I", m_modules[0].m_turnPIDController.GetI() );
    frc::SmartDashboard::PutNumber("Turn D", m_modules[0].m_turnPIDController.GetD() );

    frc::SmartDashboard::PutBoolean("Update Parameters", false );
#endif /* TUNING */

        // Reset the gyro
    ResetGyro(0_deg);

    frc::SmartDashboard::PutData("Field", &m_field);

    // fs::path deployDirectory = frc::filesystem::GetDeployDirectory();
    // deployDirectory = deployDirectory / "output" / "Example Trajectory.wpilib.json";
    // exampleTraj = frc::TrajectoryUtil::FromPathweaverJson(deployDirectory.string());

    pathplanner::AutoBuilder::configureHolonomic(
        [this](){ return GetPose(); },
        [this](frc::Pose2d pose){ ResetGyro(pose.Rotation().Degrees()); ResetPose(pose); },
        [this](){frc::ChassisSpeeds s = GetRobotRelativeSpeeds(); return s;},
        [this](frc::ChassisSpeeds speeds){ Drive(speeds, false); },
        pathplanner::HolonomicPathFollowerConfig( // HolonomicPathFollowerConfig, this should likely live in your Constants class
            pathplanner::PIDConstants(5.0, 0.0, 0.0), // Translation PID constants
            pathplanner::PIDConstants(4.0, 0.0, 0.0), // Rotation PID constants
            1.0_mps, // Max module speed, in m/s
            0.61_m, // Drive base radius in meters. Distance from robot center to furthest module.
            pathplanner::ReplanningConfig() // Default path replanning config. See the API for the options here
        ),
        []() {
            // Boolean supplier that controls when the path will be mirrored for the red alliance
            // This will flip the path being followed to the red side of the field.
            // THE ORIGIN WILL REMAIN ON THE BLUE SIDE

            auto alliance = frc::DriverStation::GetAlliance();
            if (alliance.has_value()) {
                return alliance.value() == frc::DriverStation::Alliance::kRed;
            }
                return false;
        },
        this
    );
}

// ArcadeDrive drives with joystick inputs
// This takes -1 to 1 inputs
void SwerveDriveSubsystem::ArcadeDrive( double xPercent, double yPercent, double omegaPercent, bool operatorRelative ) {
    auto x = xPercent * swerve::physical::kMaxDriveSpeed;
    auto y = yPercent * swerve::physical::kMaxDriveSpeed;
    auto omega = omegaPercent * swerve::physical::kMaxTurnSpeed;

    frc::ChassisSpeeds speeds{ x, y, omega };

    if(operatorRelative) {
        speeds = speeds.FromFieldRelativeSpeeds( speeds.vx, speeds.vy, speeds.omega, m_gyro.GetYaw().GetValue() + driver_offset);
    }

    Drive( speeds, false );
}

void SwerveDriveSubsystem::Drive( frc::ChassisSpeeds speeds, bool fieldRelative ) {
    if(fieldRelative) {
        m_robotRelativeSpeeds = speeds.FromFieldRelativeSpeeds(speeds.vx, speeds.vy, speeds.omega, m_gyro.GetYaw().GetValue());
    } else {
        m_robotRelativeSpeeds = speeds;
    }

    // An array of SwerveModuleStates computed from the ChassisSpeeds object
    // m_desiredStates = m_kinematics.ToSwerveModuleStates( fieldRelative ? speeds.FromFieldRelativeSpeeds( 
    //                 speeds.vx, speeds.vy, speeds.omega, m_gyro.GetYaw().GetValue() ) :
    //                 speeds );
    m_desiredStates = m_kinematics.ToSwerveModuleStates( m_robotRelativeSpeeds );
    m_kinematics.DesaturateWheelSpeeds( &m_desiredStates, swerve::physical::kMaxDriveSpeed );
}

// Drives a path given a trajectory state
void SwerveDriveSubsystem::DriveTrajectory( frc::Trajectory::State trajState, const frc::Rotation2d &robotHeading ) {
    // A ChassisSpeeds objects based on the current position on the trajectory
    auto adjustedSpeeds = m_controller.Calculate( m_odometry.GetEstimatedPosition(), trajState, robotHeading.Degrees() );

    Drive( adjustedSpeeds, false );
}

void SwerveDriveSubsystem::Periodic( void ) {

#ifdef TUNING
    if( frc::SmartDashboard::GetBoolean("Update Parameters", false ) ) {
        TuneSwerveDrive();
        frc::SmartDashboard::PutBoolean("Update Parameters", false );
    }

    frc::SmartDashboard::PutNumber("Front Left Absolute Position", m_modules[0].m_turnAbsEncoder.GetAbsolutePosition().GetValueAsDouble());
    frc::SmartDashboard::PutNumber("Front Right Absolute Position", m_modules[1].m_turnAbsEncoder.GetAbsolutePosition().GetValueAsDouble());
    frc::SmartDashboard::PutNumber("Back Left Absolute Position", m_modules[2].m_turnAbsEncoder.GetAbsolutePosition().GetValueAsDouble());
    frc::SmartDashboard::PutNumber("Back Right Absolute Position", m_modules[3].m_turnAbsEncoder.GetAbsolutePosition().GetValueAsDouble());

    frc::SmartDashboard::PutNumber("Turn Motor Position", m_modules[0].GetPosition().angle.Degrees().value());
    frc::SmartDashboard::PutNumber("Turn Motor Position Setpoint", m_desiredStates[0].angle.Degrees().value());
    frc::SmartDashboard::PutNumber("Drive Motor Velocity", m_modules[0].GetState().speed.value());
    frc::SmartDashboard::PutNumber("Drive Motor Velocity Setpoint", m_desiredStates[0].speed.value());

    //m_swerve_display.SetState( m_desiredStates );

#else
    for(int i = 0; i < 4; i++) {
        DataLogger::GetInstance().SendNT( m_modules[i].m_name + "/Turn Setpoint", m_modules[i].state.angle.Degrees().value() );
        DataLogger::GetInstance().SendNT( m_modules[i].m_name + "/Turn Position", m_modules[i].m_turnAbsEncoder.GetPosition().GetValueAsDouble() );
        DataLogger::GetInstance().SendNT( m_modules[i].m_name + "/Turn Raw Position", m_modules[i].m_turnAbsEncoder.GetAbsolutePosition().GetValueAsDouble() );
        DataLogger::GetInstance().SendNT( m_modules[i].m_name + "/Turn pidoutput", m_modules[i].pidOutput );

        DataLogger::GetInstance().SendNT( m_modules[i].m_name + "/Delta Theta", m_modules[i].dTheta.value() );
        DataLogger::GetInstance().SendNT( m_modules[i].m_name + "/Desired RPM", m_modules[i].speed.value() );
        DataLogger::GetInstance().SendNT( m_modules[i].m_name + "/Optimized RPM", m_modules[i].opSpeed.value() );
        DataLogger::GetInstance().SendNT( m_modules[i].m_name + "/Drive Current", m_modules[i].m_driveMotor.GetSupplyCurrent().GetValueAsDouble() );
        DataLogger::GetInstance().SendNT( m_modules[i].m_name + "/Turn Current", m_modules[i].m_turnMotor.GetSupplyCurrent().GetValueAsDouble() );
        DataLogger::GetInstance().SendNT( m_modules[i].m_name + "/Drive Motor Speed", m_modules[i].m_driveMotor.GetVelocity().GetValueAsDouble() );
        DataLogger::GetInstance().SendNT( m_modules[i].m_name + "/Turn Motor Speed", m_modules[i].m_turnMotor.GetVelocity().GetValueAsDouble() );
    }

#endif /* TUNING */

    // Sets each SwerveModule to the correct SwerveModuleState
    for( int i=0; i<4; ++i ) {
        m_modules[i].SetDesiredState( m_desiredStates[i] );
    }

    if(frc::DriverStation::IsDisabled() && !m_have_driver_offset ) {
        auto pose = m_odometry.GetEstimatedPosition();
        if(frc::DriverStation::GetAlliance().value() == frc::DriverStation::Alliance::kRed) {
            driver_offset =  pose.Rotation().Degrees() + 180_deg;
        } else {
            driver_offset = pose.Rotation().Degrees();
        }
    } else if( !frc::DriverStation::IsDisabled() && !m_have_driver_offset ) {
            // Only get a driver offset on the first enabling..
        m_have_driver_offset = true;
         fmt::print( "Stop getting offset has{} = {:.5}\n", m_have_driver_offset, driver_offset.value() );
   }

    // Updates the odometry of the robot given the SwerveModules' states
    //needs to be an array

frc::SmartDashboard::PutNumber("Gyro Angle", m_gyro.GetYaw().GetValueAsDouble() );

    m_odometry.Update( m_gyro.GetYaw().GetValue(),
    {
         m_modules[0].GetPosition(),  m_modules[1].GetPosition(), 
         m_modules[2].GetPosition(),  m_modules[3].GetPosition() 
    });

    m_vision->UpdateVisionPose( m_odometry );

    if( m_logging ) {
        // Log the swerve states
        for( int i=0; i<4; ++i ) {
            m_actualStates[i] = m_modules[i].GetState();
        }
        LogSwerveStateArray( m_actualLogEntry, m_actualStates );
        LogSwerveStateArray( m_desiredLogEntry, m_desiredStates );

        // Log the gyro angle
        m_gyroYawLogEntry.Append(m_gyro.GetAngle());

        // Log the Robot pose
        frc::Pose2d currentPose = m_odometry.GetEstimatedPosition();
        m_poseLogEntry.Append( { currentPose.X().value(), 
                                 currentPose.Y().value(), 
                                 currentPose.Rotation().Degrees().value() } );
    }

    m_field.SetRobotPose(GetPose());
}

frc::ChassisSpeeds SwerveDriveSubsystem::GetRobotRelativeSpeeds() {
    return m_robotRelativeSpeeds;
}

// Returns the pose2d of the robot
frc::Pose2d SwerveDriveSubsystem::GetPose( void ) {
    return m_odometry.GetEstimatedPosition();
}

// Resets the gyro to an angle
void SwerveDriveSubsystem::ResetGyro( units::degree_t angle ) {
    driver_offset -= angle;
    fmt::print( "   RESET GYRO, angle ({:.5}), driver_offset ({:.5})\n", angle, driver_offset );
    m_gyro.SetYaw(angle);
}

// Resets the gyro to an angle
void SwerveDriveSubsystem::ResetDriverOrientation( units::degree_t angle ) {
    driver_offset = 0_deg;
    ResetGyro(angle);
}

// Resets the pose to a position
void SwerveDriveSubsystem::ResetPose( frc::Pose2d pose ) {
    m_odometry.ResetPosition(
        m_gyro.GetYaw().GetValue(),
        {
            m_modules[0].GetPosition(),  m_modules[1].GetPosition(), 
            m_modules[2].GetPosition(),  m_modules[3].GetPosition() 
        },
        frc::Pose2d{ pose.X(), pose.Y(), m_gyro.GetYaw().GetValue() }
    );
}

void SwerveDriveSubsystem::StartLogging( wpi::log::DataLog& log ) {
    m_logging = true;
    m_actualLogEntry = wpi::log::DoubleArrayLogEntry( log, "Swerve/Actual States" );
    m_desiredLogEntry = wpi::log::DoubleArrayLogEntry( log, "Swerve/Desired States" );
    m_poseLogEntry = wpi::log::DoubleArrayLogEntry( log, "Robot/Robot2D" );
    m_gyroYawLogEntry = wpi::log::DoubleLogEntry( log, "Swerve/GyroYaw" );
}

void SwerveDriveSubsystem::LogSwerveStateArray( wpi::log::DoubleArrayLogEntry& logEntry, 
                                       wpi::array<frc::SwerveModuleState, 4U> states ) {
    static double state_array[8];

    for( int i=0; i<4; ++i ) {
        state_array[2*i] = states[i].angle.Radians().value(); 
        state_array[2*i + 1] = states[i].speed.value();
    }
    logEntry.Append( state_array );
}

void SwerveDriveSubsystem::TuneSwerveDrive() {
#ifdef TUNING
    double val;
    static units::radians_per_second_t MaxVel{ swerve::pidf::Th_Holo_MaxVel };
    static units::radians_per_second_squared_t MaxAcc{ swerve::pidf::Th_Holo_MaxAcc };

#define SET_HOLO_IF_CHANGED( name, pidc, getf, setf ) \
            val = frc::SmartDashboard::GetNumber((name), pidc.getf() ); \
            if( val != pidc.getf() ) { pidc.setf( val ); }

    SET_HOLO_IF_CHANGED( "X_Holo P", m_controller.getXController(), GetP, SetP )
    SET_HOLO_IF_CHANGED( "X_Holo I", m_controller.getXController(), GetI, SetI )
    SET_HOLO_IF_CHANGED( "X_Holo D", m_controller.getXController(), GetD, SetD )
    SET_HOLO_IF_CHANGED( "Y_Holo P", m_controller.getYController(), GetP, SetP )
    SET_HOLO_IF_CHANGED( "Y_Holo I", m_controller.getYController(), GetI, SetI )
    SET_HOLO_IF_CHANGED( "Y_Holo D", m_controller.getYController(), GetD, SetD )
    SET_HOLO_IF_CHANGED( "Th_Holo P", m_controller.getThetaController(), GetP, SetP )
    SET_HOLO_IF_CHANGED( "Th_Holo I", m_controller.getThetaController(), GetI, SetI )
    SET_HOLO_IF_CHANGED( "Th_Holo D", m_controller.getThetaController(), GetD, SetD )

    double sd_maxVel = frc::SmartDashboard::GetNumber( "Th_Holo MaxVel", swerve::pidf::Th_Holo_MaxVel.value() );
    double sd_maxAcc = frc::SmartDashboard::GetNumber( "Th_Holo MaxAcc", swerve::pidf::Th_Holo_MaxAcc.value() );
    if( sd_maxVel != MaxVel.value() || sd_maxAcc != MaxAcc.value() ) {
        MaxVel = units::radians_per_second_t{ sd_maxVel };
        MaxAcc = units::radians_per_second_squared_t{ sd_maxAcc };
        m_controller.getThetaController().SetConstraints( { MaxVel, MaxAcc } );
    }

#define SET_MODULES_IF_CHANGED( name, mods, pidc, getf, setf ) \
            val = frc::SmartDashboard::GetNumber((name), mods[0].pidc.getf() ); \
            if( val != mods[0].pidc.getf() ) { \
                for(int i=0; i<4; ++i ) { \
                    mods[i].pidc.setf( val ); \
                } \
            }

    // SET_MODULES_IF_CHANGED( "Drive P", m_modules, m_drivePIDController, GetP, SetP )
    // SET_MODULES_IF_CHANGED( "Drive I", m_modules, m_drivePIDController, GetI, SetI )
    // SET_MODULES_IF_CHANGED( "Drive D", m_modules, m_drivePIDController, GetD, SetD )
    // SET_MODULES_IF_CHANGED( "Drive FF", m_modules, m_drivePIDController, GetFF, SetFF )
    SET_MODULES_IF_CHANGED( "Turn P", m_modules, m_turnPIDController, GetP, SetP )
    SET_MODULES_IF_CHANGED( "Turn I", m_modules, m_turnPIDController, GetI, SetI )
    SET_MODULES_IF_CHANGED( "Turn D", m_modules, m_turnPIDController, GetD, SetD )
#endif /* TUNING */
}
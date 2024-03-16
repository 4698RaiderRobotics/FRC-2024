//
//      Base Class for a logged robot to write basic data to a log file.
//

#include <frc/RobotController.h>
#include <frc/DataLogManager.h>
#include <frc/DriverStation.h>
#include <frc/livewindow/LiveWindow.h>
#include <frc2/command/CommandScheduler.h>
#include <frc2/command/Command.h>

#include "DataLogger.h"
#include "LoggedRobot.h"

void LoggedRobot::RobotInit() {
        // Disable LiveWindow Telemetry
    frc::LiveWindow::DisableAllTelemetry();
    
        // Start the log manager
    frc::DataLogManager::Start();

        // Record both DS control and joystick data
    frc::DriverStation::StartDataLog( frc::DataLogManager::GetLog() );

        // Send the metadata from the buildinfo.txt file
    DataLogger::GetInstance().LogMetadata();

        // Determine the number of PDP channels
    m_pdpChannels = (m_pdp.GetType() == frc::PowerDistribution::ModuleType::kRev) ? 24 : 16;

    frc2::CommandScheduler::GetInstance().OnCommandInitialize(  
        [](const frc2::Command& command) {
            DataLogger::GetInstance().Log( "Command " + command.GetName() + 
                                        " starting..." );
            DataLogger::GetInstance().SendNT( "Command/" + command.GetName(), true );
        }
        );
    frc2::CommandScheduler::GetInstance().OnCommandFinish(  
        [](const frc2::Command& command) {
            DataLogger::GetInstance().Log( "Command " + command.GetName() + 
                                        " finished." );
            DataLogger::GetInstance().SendNT( "Command/" + command.GetName(), false );
        }
    );
    frc2::CommandScheduler::GetInstance().OnCommandInterrupt(  
        [](const frc2::Command& command, const std::optional<frc2::Command*>& int_cmd) {
            DataLogger::GetInstance().Log( "Command " + command.GetName() + 
                                        " interrupted by " + (int_cmd.has_value() ? int_cmd.value()->GetName() : "Disabled?") );
        }
    );
}

void LoggedRobot::RobotPeriodic() {
    static double currents[24];
    frc::PowerDistribution::Faults faults;

    DataLogger &logger = DataLogger::GetInstance();

    frc::CANStatus cs;

        // Log the RoboRIO Information
    logger.Send( "RoboRIO/Input Voltage", frc::RobotController::GetInputVoltage() );
    logger.Send( "RoboRIO/Input Current", frc::RobotController::GetInputCurrent() );
    logger.Send( "RoboRIO/BrownedOut", frc::RobotController::IsBrownedOut() );
    logger.Send( "RoboRIO/3V3 Volts", frc::RobotController::GetVoltage3V3() );
    logger.Send( "RoboRIO/3V3 Amps", frc::RobotController::GetCurrent3V3() );
    logger.Send( "RoboRIO/3V3 Fault Count", frc::RobotController::GetFaultCount3V3() );
    logger.Send( "RoboRIO/5V Volts", frc::RobotController::GetVoltage5V() );
    logger.Send( "RoboRIO/5V Amps", frc::RobotController::GetCurrent5V() );
    logger.Send( "RoboRIO/5V Fault Count", frc::RobotController::GetFaultCount5V() );

        // Log the RoboRIO CANBus Stats
    cs = frc::RobotController::GetCANStatus();
    logger.Send( "RoboRIO/CAN Percent Utilization", cs.percentBusUtilization );
    logger.Send( "RoboRIO/CAN OffCount", cs.busOffCount );
    logger.Send( "RoboRIO/CAN receiveErrorCount", cs.receiveErrorCount );
    logger.Send( "RoboRIO/CAN transmitErrorCount", cs.transmitErrorCount );
    logger.Send( "RoboRIO/CAN txFullCount", cs.txFullCount );

        // Log the PDP Information
    faults = m_pdp.GetFaults();

    logger.Send( "PDP/Bus Voltage", m_pdp.GetVoltage() );
    logger.Send( "PDP/Total Current", m_pdp.GetTotalCurrent() );
    logger.Send( "PDP/Temperature", m_pdp.GetTemperature() );
    logger.Send( "PDP/Total Power", m_pdp.GetTotalPower() );
    logger.Send( "PDP/Total Energy", m_pdp.GetTotalEnergy() );
    logger.Send( "PDP/Brown Out", (bool) faults.Brownout );
    logger.Send( "PDP/Can Warning", (bool) faults.CanWarning );
    logger.Send( "PDP/Hardware Fault", (bool) faults.HardwareFault );

    for( int i=0; i<m_pdpChannels; ++i ) {
        currents[i] = m_pdp.GetCurrent( i );
    }
    logger.Send( "PDP/Currents", std::span<double> ( currents, m_pdpChannels) );
}

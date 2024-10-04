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
    DataLogger::LogMetadata();

        // Determine the number of PDP channels
    m_pdpChannels = (m_pdp.GetType() == frc::PowerDistribution::ModuleType::kRev) ? 24 : 16;

    frc2::CommandScheduler::GetInstance().OnCommandInitialize(  
        [](const frc2::Command& command) {
            DataLogger::Log( "Command " + command.GetName() + 
                                        " starting..." );
            DataLogger::SendNT( "Command/" + command.GetName(), true );
        }
        );
    frc2::CommandScheduler::GetInstance().OnCommandFinish(  
        [](const frc2::Command& command) {
            DataLogger::Log( "Command " + command.GetName() + 
                                        " finished." );
            DataLogger::SendNT( "Command/" + command.GetName(), false );
        }
    );
    frc2::CommandScheduler::GetInstance().OnCommandInterrupt(  
        [](const frc2::Command& command, const std::optional<frc2::Command*>& int_cmd) {
            DataLogger::SendNT( "Command/" + command.GetName(), false );
            DataLogger::Log( "Command <" + command.GetName() + 
                                        "> interrupted by <" + (int_cmd.has_value() ? int_cmd.value()->GetName() : "<DISABLED>") + ">" );
        }
    );
}

void LoggedRobot::RobotPeriodic() {
    static double currents[24];
    frc::PowerDistribution::Faults faults;

    frc::CANStatus cs;

        // Log the RoboRIO Information
     DataLogger::SendNT( "RoboRIO/Input Voltage", frc::RobotController::GetInputVoltage() );
     DataLogger::SendNT( "RoboRIO/Input Current", frc::RobotController::GetInputCurrent() );
     DataLogger::SendNT( "RoboRIO/BrownedOut", frc::RobotController::IsBrownedOut() );
     DataLogger::SendNT( "RoboRIO/3V3 Volts", frc::RobotController::GetVoltage3V3() );
     DataLogger::SendNT( "RoboRIO/3V3 Amps", frc::RobotController::GetCurrent3V3() );
     DataLogger::SendNT( "RoboRIO/3V3 Fault Count", frc::RobotController::GetFaultCount3V3() );
     DataLogger::SendNT( "RoboRIO/5V Volts", frc::RobotController::GetVoltage5V() );
     DataLogger::SendNT( "RoboRIO/5V Amps", frc::RobotController::GetCurrent5V() );
     DataLogger::SendNT( "RoboRIO/5V Fault Count", frc::RobotController::GetFaultCount5V() );

        // Log the RoboRIO CANBus Stats
    cs = frc::RobotController::GetCANStatus();
     DataLogger::SendNT( "RoboRIO/CAN Percent Utilization", cs.percentBusUtilization );
     DataLogger::SendNT( "RoboRIO/CAN OffCount", cs.busOffCount );
     DataLogger::SendNT( "RoboRIO/CAN receiveErrorCount", cs.receiveErrorCount );
     DataLogger::SendNT( "RoboRIO/CAN transmitErrorCount", cs.transmitErrorCount );
     DataLogger::SendNT( "RoboRIO/CAN txFullCount", cs.txFullCount );

        // Log the PDP Information
    faults = m_pdp.GetFaults();

     DataLogger::SendNT( "PDP/Bus Voltage", m_pdp.GetVoltage() );
     DataLogger::SendNT( "PDP/Total Current", m_pdp.GetTotalCurrent() );
     DataLogger::SendNT( "PDP/Temperature", m_pdp.GetTemperature() );
     DataLogger::SendNT( "PDP/Total Power", m_pdp.GetTotalPower() );
     DataLogger::SendNT( "PDP/Total Energy", m_pdp.GetTotalEnergy() );
     DataLogger::SendNT( "PDP/Brown Out", (bool) faults.Brownout );
     DataLogger::SendNT( "PDP/Can Warning", (bool) faults.CanWarning );
     DataLogger::SendNT( "PDP/Hardware Fault", (bool) faults.HardwareFault );

    for( int i=0; i<m_pdpChannels; ++i ) {
        currents[i] = m_pdp.GetCurrent( i );
    }
    DataLogger::SendNT( "PDP/Currents", std::span<double> ( currents, m_pdpChannels) );
}

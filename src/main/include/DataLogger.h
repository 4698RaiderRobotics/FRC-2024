// Log data to the Data Log file on the RoboRIO
//

#pragma once

#include <span>
#include <map>

#include <wpi/DataLog.h>
#include <frc/DataLogManager.h>
#include <networktables/NetworkTable.h>
#include <networktables/NetworkTableInstance.h>
#include <networktables/GenericEntry.h>

#include <frc/geometry/Pose2d.h>

class DataLogger {
  private:
      // This class is a singleton.
    static DataLogger *singleton;

      // Constructor is private
    DataLogger() {}
    static DataLogger& GetInstance();

  public:
      // delete copy constructor
    DataLogger(const DataLogger& obj) = delete; 

    static void Send( const std::string &s, double val );
    static void Send( const std::string &s, std::span<const double> a );
    static void Send( const std::string &s, const std::string &val );
    static void Send( const std::string &s, int val );
    static void Send( const std::string &s, bool val );
    static void Send( const std::string &s, frc::Pose2d p );

    static void SendNT( const std::string &s, double val );
    static void SendNT( const std::string &s, std::span<const double> a );
    static void SendNT( const std::string &s, const std::string &val );
    static void SendNT( const std::string &s, int val );
    static void SendNT( const std::string &s, bool val );
    static void SendNT( const std::string &s, frc::Pose2d p );

    static void Log( const std::string &s );
    static void LogMetadata( void );

  private:
    wpi::log::DataLog *log;
    std::shared_ptr<nt::NetworkTable> nt_table;
    std::map<std::string, nt::GenericPublisher> nt_map;

    void SendMetadata( const std::string &s, const std::string &val );

};

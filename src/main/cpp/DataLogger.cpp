// Log data to the Data Log file on the RoboRIO
//

#include <fstream>

#include <wpi/DataLog.h>
#include <frc/Filesystem.h>
#include <frc/DataLogManager.h>
#include <networktables/DoubleTopic.h>
#include <networktables/DoubleArrayTopic.h>
#include <networktables/IntegerTopic.h>
#include <networktables/BooleanTopic.h>
#include <networktables/StringTopic.h>

#include "DataLogger.h"

DataLogger* DataLogger::singleton = nullptr; 

DataLogger& DataLogger::GetInstance() {
        // If there is no instance of class
        // then we can create an instance.
    if (singleton == nullptr)  {
        singleton = new DataLogger();
        singleton->log = &frc::DataLogManager::GetLog();
        singleton->nt_table = nt::NetworkTableInstance::GetDefault().GetTable("");
    }
        
    return *singleton;
}

void DataLogger::Send( const std::string &s, double val ) { 
    wpi::log::DoubleLogEntry le{ *(GetInstance().log), s };
    le.Append( val );
}

void DataLogger::Send( const std::string &s, std::span<const double> a ) { 
    wpi::log::DoubleArrayLogEntry le{ *(GetInstance().log), s };
    le.Append( a );
}

void DataLogger::Send( const std::string &s, int val ) { 
    wpi::log::IntegerLogEntry le{ *(GetInstance().log), s };
    le.Append( val );
}

void DataLogger::Send( const std::string &s, const std::string &val ) { 
    wpi::log::StringLogEntry le{ *(GetInstance().log), s };
    le.Append( val );
}

void DataLogger::Send( const std::string &s, bool val ) {
    wpi::log::BooleanLogEntry le{ *(GetInstance().log), s };
    le.Append( val );
}

void DataLogger::Send( const std::string &s, frc::Pose2d p ) {
    wpi::log::DoubleArrayLogEntry le{ *(GetInstance().log), s };
    le.Append( {p.X().value(),
                p.Y().value(),
                p.Rotation().Degrees().value()} );
}

void DataLogger::SendNT( const std::string &s, double val ) {
    if( !GetInstance().nt_map.contains( s ) ) {
        GetInstance().nt_map[s] = GetInstance().nt_table->GetDoubleTopic( s ).GenericPublish( "double" );
    }
    GetInstance().nt_map[s].SetDouble( val );
}

void DataLogger::SendNT( const std::string &s, std::span<const double> a ) {
    if( !GetInstance().nt_map.contains( s ) ) {
        GetInstance().nt_map[s] = GetInstance().nt_table->GetDoubleArrayTopic( s ).GenericPublish( "double[]" );
    }
    GetInstance().nt_map[s].SetDoubleArray( a );
}

void DataLogger::SendNT( const std::string &s, const std::string &val ) {
    if( !GetInstance().nt_map.contains( s ) ) {
        GetInstance().nt_map[s] = GetInstance().nt_table->GetStringTopic( s ).GenericPublish( "string" );
    }
    GetInstance().nt_map[s].SetString( val );
}

void DataLogger::SendNT( const std::string &s, int val ) {
    if( !GetInstance().nt_map.contains( s ) ) {
        GetInstance().nt_map[s] = GetInstance().nt_table->GetIntegerTopic( s ).GenericPublish( "integer" );
    }
    GetInstance().nt_map[s].SetInteger( val );
}

void DataLogger::SendNT( const std::string &s, bool val ) {
    if( !GetInstance().nt_map.contains( s ) ) {
        GetInstance().nt_map[s] = GetInstance().nt_table->GetBooleanTopic( s ).GenericPublish( "boolean" );
    }
    GetInstance().nt_map[s].SetBoolean( val );
}

void DataLogger::SendNT( const std::string &s, frc::Pose2d p ) {
    static double a[3];
    
    a[0] = p.X().value();
    a[1] = p.Y().value();
    a[2] = p.Rotation().Degrees().value();

    SendNT( s, std::span{a} );
}

void DataLogger::Log( const std::string &s ) {
    frc::DataLogManager::Log( s );
}

void DataLogger::LogMetadata( void ) {
        // Open the buildinfo.txt file and write the Metadata to the log file
    std::ifstream binfo;
    char line[256];

    std::string path = frc::filesystem::GetDeployDirectory() + "/buildinfo.txt";

    binfo.open( path, std::ios::in );
    if( binfo.is_open() ) {
        binfo.getline( line, 255 );
        GetInstance().SendMetadata( "BUILD_DATE", line );
        binfo.getline( line, 255 );
        GetInstance().SendMetadata( "GIT_REPO", line );
        binfo.getline( line, 255 );
        GetInstance().SendMetadata( "GIT_BRANCH", line );
        binfo.getline( line, 255 );
        GetInstance().SendMetadata( "GIT_VERSION", line );
        binfo.close();
    } else {
        Log( "Cannot open METADATA file: " + path );
    }

}

void DataLogger::SendMetadata( const std::string &s, const std::string &val ) {
        // AdvantageScope Chops off leading Character of the name so we add an underscore.
        // Not sure why
    std::string id = "RealMetadata/_";
    id += s;
    wpi::log::StringLogEntry le{ *(log), id };
    le.Append( val );
}

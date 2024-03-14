// Log data to the Data Log file on the RoboRIO
//

#pragma once

#include <frc2/command/Command.h>
#include <frc2/command/CommandHelper.h>

#include "DataLogger.h"

class LoggedCommand : public frc2::Command {
public:
  

  void Initialize() final {
    DataLogger::GetInstance().Send( "Command/" + this->GetName(), true );
    DataLogger::GetInstance().Log( "   Command " +  this->GetName() + " initialized" );
    Init();
  }
  void End( bool interrupted ) final {
    HasEnded( interrupted );
    DataLogger::GetInstance().Send( "Command/" + this->GetName(), false );
    DataLogger::GetInstance().Log( "   Command " +  this->GetName() + " ended" );
  }

  virtual void Init() {}
  virtual void HasEnded( bool interrupted ) {}
};

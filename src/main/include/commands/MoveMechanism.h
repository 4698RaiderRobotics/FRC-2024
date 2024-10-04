// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc2/command/Command.h>
#include <frc2/command/CommandHelper.h>

#include "LoggedCommand.h"

class ArmSubsystem;
class ElevatorSubsystem;

/**
 * An example command.
 *
 * <p>Note that this extends CommandHelper, rather extending Command
 * directly; this is crucially important, or else the decorator functions in
 * Command will *not* work!
 */
class MoveMechanism
    : public frc2::CommandHelper<LoggedCommand, MoveMechanism> {
 public:
  MoveMechanism( ArmSubsystem *arm, ElevatorSubsystem *elev,
                 units::degree_t arm_pos, units::degree_t wrist_pos, units::meter_t height );

  void Init() override;

  void Execute() override;

  void Ending(bool interrupted) override;

  bool IsFinished() override;

 private:
    // Returns true when completed
  bool FlipArmForward();

    // Returns true when completed
  bool FlipArmBackward();


  ArmSubsystem *m_arm;
  ElevatorSubsystem *m_elev;

  units::degree_t m_arm_target;
  units::degree_t m_wrist_target;
  units::meter_t m_height_target;

  bool m_elev_going_up;
  bool m_elev_going_down;
  bool m_arm_going_forward;
  bool m_arm_going_back;
  bool m_wrist_retracted;
  bool m_elev_is_done;
};

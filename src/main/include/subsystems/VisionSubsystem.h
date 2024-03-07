// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc2/command/SubsystemBase.h>
#include <frc/apriltag/AprilTagFieldLayout.h>

#include <photon/PhotonCamera.h>
#include <photon/PhotonPoseEstimator.h>

#include "Constants.h"

class VisionSubsystem : public frc2::SubsystemBase {
 public:
  VisionSubsystem();

  /**
   * Will be called periodically whenever the CommandScheduler runs.
   */
  void Periodic() override;

  std::pair<frc::Pose2d, units::second_t> GetGlobalEstimatedPose();

  bool GetAprilTagPosition( int tagID, double &yaw, double &pitch );

 private:
  photon::PhotonCamera m_camera{"ShooterCam"};
  photon::PhotonPoseEstimator *m_robotPoseEstimator;
  photon::PhotonPipelineResult m_result;
  units::second_t m_lastestTargetTime;

 public:
  frc::AprilTagFieldLayout aprilTagFieldLayout;
};

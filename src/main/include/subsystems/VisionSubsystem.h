// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc2/command/SubsystemBase.h>
#include <frc/apriltag/AprilTagFieldLayout.h>
#include <frc/estimator/SwerveDrivePoseEstimator.h>

#include <photon/PhotonCamera.h>
#include <photon/PhotonPoseEstimator.h>

#include "Constants.h"

class VisionSubsystem : public frc2::SubsystemBase {
 public:
  VisionSubsystem();

  void Periodic() override;

  void UpdateVisionPose(frc::SwerveDrivePoseEstimator<4> &odometry);

 private:
  photon::PhotonCamera m_frontRightCam{"CameraA_1MP"};
  photon::PhotonPoseEstimator *m_frontRightPoseEstimator;
  photon::PhotonCamera m_backRightCam{"CameraB_2MP"};
  photon::PhotonPoseEstimator *m_backRightPoseEstimator;
  // photon::PhotonCamera m_frontLeftCam{"FrontLeftCam"};
  // photon::PhotonPoseEstimator *m_frontLeftPoseEstimator;
  // photon::PhotonCamera m_backLeftCam{"BackLeftCam"};
  // photon::PhotonPoseEstimator *m_backLeftPoseEstimator;

};

// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "subsystems/VisionSubsystem.h"

#include <frc/Timer.h>

VisionSubsystem::VisionSubsystem() 
: m_robotPoseEstimator{aprilTagFieldLayout, photon::PoseStrategy::MULTI_TAG_PNP_ON_COPROCESSOR, physical::kRobotToCam} {
    aprilTagFieldLayout = frc::LoadAprilTagLayoutField(frc::AprilTagField::k2024Crescendo);
    
};

// This method will be called once per scheduler run
void VisionSubsystem::Periodic() {
    // photon::PhotonPipelineResult result = m_camera.GetLatestResult();

    // bool hasTarget = result.HasTargets();

    
    
}

// std::pair<frc::Pose2d, units::second_t> VisionSubsystem::GetGlobalEstimatedPose() {
//     // units::millisecond_t currentTime = frc::Timer::GetFPGATimestamp();
//     // auto result = m_robotPoseEstimator.Update();
//     // if (result.has_value()) {
//     //     return std::make_pair<>(result->estimatedPose.ToPose2d(),
//     //                             currentTime - result->timestamp);
//     // } else {
//     //     return std::make_pair(frc::Pose2d(), 0_ms);
//     // }
// }
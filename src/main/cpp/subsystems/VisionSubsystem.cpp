// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "subsystems/VisionSubsystem.h"

#include <frc/Timer.h>

VisionSubsystem::VisionSubsystem() {
    aprilTagFieldLayout = frc::LoadAprilTagLayoutField(frc::AprilTagField::k2024Crescendo);
    m_robotPoseEstimator = new photon::PhotonPoseEstimator(aprilTagFieldLayout, photon::PoseStrategy::MULTI_TAG_PNP_ON_COPROCESSOR, physical::kRobotToCam);
};

// This method will be called once per scheduler run
void VisionSubsystem::Periodic() {
    m_result = m_camera.GetLatestResult();

    if( m_result.HasTargets() ) {
        m_lastestTargetTime = frc::Timer::GetFPGATimestamp();
        auto camtotarget = m_result.GetBestTarget().GetBestCameraToTarget();
        // fmt::print("Camera To Target: {}, {}, {}, -- {}, {}, {}\n", camtotarget.X(),camtotarget.Y(), camtotarget.Z(), 
        //                                      camtotarget.Rotation().X().convert<units::degree>().value(), 
        //                                      camtotarget.Rotation().Y().convert<units::degree>().value(),
        //                                      camtotarget.Rotation().Z().convert<units::degree>().value() );
    }

//    auto pose = m_robotPoseEstimator.Update( m_result );
}

std::pair<frc::Pose2d, units::second_t> VisionSubsystem::GetGlobalEstimatedPose() {
    if( m_result.HasTargets() ) {
        auto pose = m_robotPoseEstimator->Update( m_result );
        // fmt::print( "robotPoseEstimator pose {} {} -- {}\n", pose->estimatedPose.ToPose2d().X(), 
        //                                                      pose->estimatedPose.ToPose2d().Y(), 
        //                                                      pose->estimatedPose.ToPose2d().Rotation().Degrees());
        return std::make_pair<>(pose->estimatedPose.ToPose2d(),  pose->timestamp);
                                // frc::Timer::GetFPGATimestamp() - pose->timestamp);
    } else {
        return std::make_pair(frc::Pose2d(), 0_ms);
    }
}

bool VisionSubsystem::GetAprilTagPosition( int tagID, double &yaw, double &pitch ) {
//    fmt::print( "Has targets({}), elaspsed time {}\n", m_result.HasTargets(), frc::Timer::GetFPGATimestamp() - m_lastestTargetTime );
    if( m_result.HasTargets() && frc::Timer::GetFPGATimestamp() - m_lastestTargetTime < 2_s ) {
        // We have a target that was found within 2 seconds
        auto targets = m_result.GetTargets();
        for( auto target : targets ) {
            if( target.GetFiducialId() == tagID ) {
                yaw = target.GetYaw();
                pitch = target.GetPitch();
                return true;
            }
        }
    }

    return false;
}

// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "subsystems/VisionSubsystem.h"

#include <frc/Timer.h>

VisionSubsystem::VisionSubsystem() {
    aprilTagFieldLayout = frc::LoadAprilTagLayoutField(frc::AprilTagField::k2024Crescendo);
    m_frontRightPoseEstimator = new photon::PhotonPoseEstimator(aprilTagFieldLayout, photon::PoseStrategy::MULTI_TAG_PNP_ON_COPROCESSOR, 
                                                                physical::kFrontRightRobotToCam);
    m_backRightPoseEstimator = new photon::PhotonPoseEstimator(aprilTagFieldLayout, photon::PoseStrategy::MULTI_TAG_PNP_ON_COPROCESSOR, 
                                                                physical::kBackRightRobotToCam);
    
    // m_frontLeftPoseEstimator = new photon::PhotonPoseEstimator(aprilTagFieldLayout, photon::PoseStrategy::MULTI_TAG_PNP_ON_COPROCESSOR, physical::kFrontLeftRobotToCam);
    // m_backLeftPoseEstimator = new photon::PhotonPoseEstimator(aprilTagFieldLayout, photon::PoseStrategy::MULTI_TAG_PNP_ON_COPROCESSOR, physical::kBackLeftRobotToCam);
};

// This method will be called once per scheduler run
void VisionSubsystem::Periodic() {
    m_result = m_frontRightCam.GetLatestResult();

    if( m_result.HasTargets() ) {
        m_latestTargetTime = frc::Timer::GetFPGATimestamp();
        // auto camtotarget = m_result.GetBestTarget().GetBestCameraToTarget();
        // fmt::print("Camera To Target: {}, {}, {}, -- {}, {}, {}\n", camtotarget.X(),camtotarget.Y(), camtotarget.Z(), 
        //                                      camtotarget.Rotation().X().convert<units::degree>().value(), 
        //                                      camtotarget.Rotation().Y().convert<units::degree>().value(),
        //                                      camtotarget.Rotation().Z().convert<units::degree>().value() );
    }

//    auto pose = m_robotPoseEstimator.Update( m_result );
}

void VisionSubsystem::UpdateVisionPose(frc::SwerveDrivePoseEstimator<4> &odometry) {
    // Standard Deviations are found with a quadratic equation relating distance to target
    // to the s.d. 
    // Used the points (1,0.5), (2,0.75), (4,2) which gave the quadratic 0.125x2 - 0.125x + 0.5
    photon::PhotonPipelineResult result;
    result = m_frontRightCam.GetLatestResult();
    if( result.HasTargets() ) {
        auto pose = m_frontRightPoseEstimator->Update(result);
        if( pose.has_value() ) {
            double dist_to_tag = result.GetBestTarget().GetBestCameraToTarget().Translation().Norm().value();
            double sd_dev = (0.125*dist_to_tag - 0.125)*dist_to_tag + 0.5;
    // fmt::print( "{} updating with pose ({}, {}, {}) with std_dev {}\n", 
    // m_frontRightCam.GetCameraName(), pose->estimatedPose.ToPose2d().X(),pose->estimatedPose.ToPose2d().Y(),
    // pose->estimatedPose.ToPose2d().Rotation().Degrees(), sd_dev );
            odometry.SetVisionMeasurementStdDevs( {sd_dev, sd_dev, sd_dev} );
            odometry.AddVisionMeasurement( pose->estimatedPose.ToPose2d(), pose->timestamp );
        }
    }

    result = m_backRightCam.GetLatestResult();
// fmt::print( "Checking if {} has a target...\n", m_backRightCam.GetCameraName());
    if( result.HasTargets() ) {
// fmt::print( "     {} has a target...\n", m_backRightCam.GetCameraName());
        auto pose = m_backRightPoseEstimator->Update(result);
        if( pose.has_value() ) {
            double dist_to_tag = result.GetBestTarget().GetBestCameraToTarget().Translation().Norm().value();
            double sd_dev = (0.125*dist_to_tag - 0.125)*dist_to_tag + 0.5;
//    fmt::print( "{} updating with pose ({}, {}, {}) with std_dev {}\n", 
//     m_backRightCam.GetCameraName(), pose->estimatedPose.ToPose2d().X(),pose->estimatedPose.ToPose2d().Y(),
    // pose->estimatedPose.ToPose2d().Rotation().Degrees(), sd_dev );
            odometry.SetVisionMeasurementStdDevs( {sd_dev, sd_dev, sd_dev} );
            odometry.AddVisionMeasurement( pose->estimatedPose.ToPose2d(), pose->timestamp );
        }
    }
}

std::pair<frc::Pose2d, units::second_t> VisionSubsystem::GetGlobalEstimatedPose() {
    if( m_result.HasTargets() && m_result.GetBestTarget().GetBestCameraToTarget().Translation().Norm() < 4_m) {
        auto pose = m_frontRightPoseEstimator->Update( m_result );
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
    if( m_result.HasTargets() && frc::Timer::GetFPGATimestamp() - m_latestTargetTime < 2_s ) {
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

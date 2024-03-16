// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include <frc/Timer.h>

#include "DataLogger.h"
#include "subsystems/VisionSubsystem.h"

void UpdatePoseEstimator( photon::PhotonCamera &cam, 
                          photon::PhotonPoseEstimator *estimator, 
                          frc::SwerveDrivePoseEstimator<4> &odometry );


VisionSubsystem::VisionSubsystem() {
    frc::AprilTagFieldLayout aprilTagFieldLayout = frc::LoadAprilTagLayoutField(frc::AprilTagField::k2024Crescendo);
    m_frontRightPoseEstimator = new photon::PhotonPoseEstimator(aprilTagFieldLayout, photon::PoseStrategy::MULTI_TAG_PNP_ON_COPROCESSOR, 
                                                                physical::kFrontRightRobotToCam);
    m_backRightPoseEstimator = new photon::PhotonPoseEstimator(aprilTagFieldLayout, photon::PoseStrategy::MULTI_TAG_PNP_ON_COPROCESSOR, 
                                                                physical::kBackRightRobotToCam);
    
}

// This method will be called once per scheduler run
void VisionSubsystem::Periodic() {
    // m_result = m_frontRightCam.GetLatestResult();

    // if( m_result.HasTargets() ) {
    //     m_latestTargetTime = frc::Timer::GetFPGATimestamp();
    //     // auto camtotarget = m_result.GetBestTarget().GetBestCameraToTarget();
    //     // fmt::print("Camera To Target: {}, {}, {}, -- {}, {}, {}\n", camtotarget.X(),camtotarget.Y(), camtotarget.Z(), 
    //     //                                      camtotarget.Rotation().X().convert<units::degree>().value(), 
    //     //                                      camtotarget.Rotation().Y().convert<units::degree>().value(),
    //     //                                      camtotarget.Rotation().Z().convert<units::degree>().value() );
    // }

//    auto pose = m_robotPoseEstimator.Update( m_result );
}

void VisionSubsystem::UpdateVisionPose(frc::SwerveDrivePoseEstimator<4> &odometry) {

    UpdatePoseEstimator( m_frontRightCam, m_frontRightPoseEstimator, odometry );
    UpdatePoseEstimator( m_backRightCam, m_backRightPoseEstimator, odometry );

}

void UpdatePoseEstimator( photon::PhotonCamera &cam, 
                          photon::PhotonPoseEstimator *estimator, 
                          frc::SwerveDrivePoseEstimator<4> &odometry ) {

    // Standard Deviations are found with a quadratic equation relating distance to target
    // to the s.d. 
    // Used the points (1,0.5), (2,0.75), (4,2) which gave the quadratic 0.125x2 - 0.125x + 0.5

    photon::PhotonPipelineResult result;
    result = cam.GetLatestResult();
    if( result.HasTargets() ) {
        std::string id = "Vision/";
        id += cam.GetCameraName();
        id += "/Pose2d";

        auto pose = estimator->Update(result);
        if( pose.has_value() ) {

            DataLogger::GetInstance().Send( id, pose->estimatedPose.ToPose2d() );

            double dist_to_tag = result.GetBestTarget().GetBestCameraToTarget().Translation().Norm().value();
            double sd_dev = (0.125*dist_to_tag - 0.125)*dist_to_tag + 0.5;
    // fmt::print( "{} updating with pose ({}, {}, {}) with std_dev {}\n", 
    // m_frontRightCam.GetCameraName(), pose->estimatedPose.ToPose2d().X(),pose->estimatedPose.ToPose2d().Y(),
    // pose->estimatedPose.ToPose2d().Rotation().Degrees(), sd_dev );
            odometry.SetVisionMeasurementStdDevs( {sd_dev, sd_dev, sd_dev} );
            odometry.AddVisionMeasurement( pose->estimatedPose.ToPose2d(), pose->timestamp );
        } else {
                // Log -20, -20, 0
            DataLogger::GetInstance().Send( id, frc::Pose2d{-20_m, -20_m, 0_deg} );
        }
    }

}

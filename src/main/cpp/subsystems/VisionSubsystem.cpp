// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include <frc/Timer.h>
#include <frc/DriverStation.h>

#include "DataLogger.h"
#include "subsystems/VisionSubsystem.h"

void UpdatePoseEstimator( std::string camName, 
                          photon::PhotonPoseEstimator &estimator, 
                          frc::SwerveDrivePoseEstimator<4> &odometry );


VisionSubsystem::VisionSubsystem() : 
    m_frontRightPoseEstimator{ frc::LoadAprilTagLayoutField(frc::AprilTagField::k2024Crescendo),
                               photon::PoseStrategy::MULTI_TAG_PNP_ON_COPROCESSOR,
                               std::move( photon::PhotonCamera{m_frontRightCameraName} ),
                               physical::kFrontRightRobotToCam },
    m_backRightPoseEstimator{ frc::LoadAprilTagLayoutField(frc::AprilTagField::k2024Crescendo),
                               photon::PoseStrategy::MULTI_TAG_PNP_ON_COPROCESSOR,
                               std::move( photon::PhotonCamera{m_backRightCameraName} ),
                               physical::kBackRightRobotToCam },
    m_backLeftPoseEstimator{ frc::LoadAprilTagLayoutField(frc::AprilTagField::k2024Crescendo),
                               photon::PoseStrategy::MULTI_TAG_PNP_ON_COPROCESSOR,
                               std::move( photon::PhotonCamera{m_backLeftCameraName} ),
                               physical::kBackLeftRobotToCam }
    
 {
 
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

    UpdatePoseEstimator( m_frontRightCameraName, m_frontRightPoseEstimator, odometry );
    UpdatePoseEstimator( m_backRightCameraName, m_backRightPoseEstimator, odometry );
    UpdatePoseEstimator( m_backLeftCameraName, m_backLeftPoseEstimator, odometry );

}

void UpdatePoseEstimator( std::string camName, 
                          photon::PhotonPoseEstimator &estimator, 
                          frc::SwerveDrivePoseEstimator<4> &odometry ) {

    // Standard Deviations are found with a quadratic equation relating distance to target
    // to the s.d. 
    // Used the points (1,0.5), (2,0.75), (4,6) which gave the quadratic 0.792x2 - 2.125x + 1.833

    std::string id_base = "Vision/";
    id_base += camName;
    id_base += "/";

    auto result = estimator.GetCamera()->GetLatestResult();

    if( result.HasTargets() ) {

        auto pose = estimator.Update( result );
        if( pose.has_value() && result.MultiTagResult().result.ambiguity < 0.2 ) {

            double dist_to_tag = result.GetBestTarget().GetBestCameraToTarget().Translation().Norm().value();
            double sd_dev = (0.792*dist_to_tag - 2.125)*dist_to_tag + 1.833;
            if(dist_to_tag < 1.3) {
                sd_dev = 0.4;
            }

            DataLogger::SendNT( id_base + "Pose2d", pose->estimatedPose.ToPose2d() );
            DataLogger::SendNT( id_base + "Stdev", sd_dev );

            if(frc::DriverStation::IsEnabled() && odometry.GetEstimatedPosition().RelativeTo(pose->estimatedPose.ToPose2d()).Translation().Norm() > 3_m) {
                return;
            }
            odometry.SetVisionMeasurementStdDevs( {sd_dev, sd_dev, sd_dev} );
            odometry.AddVisionMeasurement( pose->estimatedPose.ToPose2d(), pose->timestamp );
                // Log the pose
            
        } else {
                // We got a bad pose. Log -10, -10, 0
            DataLogger::SendNT( id_base + "Pose2d", frc::Pose2d{-10_m, -10_m, 0_deg} );
        }
    } else {
            // We don't have a result.  Log -20, -20, 0
        DataLogger::SendNT( id_base + "Pose2d", frc::Pose2d{-20_m, -20_m, 0_deg} );
    }

}

frc::Pose2d VisionSubsystem::GetRelativePose() {
    auto result = m_frontRightPoseEstimator.GetCamera()->GetLatestResult();

    if(result.HasTargets()) {
        photon::PhotonTrackedTarget target = result.GetBestTarget();

        if(target.GetPoseAmbiguity() < 0.2) {
            return frc::Pose3d{target.GetBestCameraToTarget().Translation(), target.GetBestCameraToTarget().Rotation()}.ToPose2d();
        }
    }
}
// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.lib.util;

import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.targeting.PhotonPipelineResult;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.networktables.NetworkTableInstance;

/** Add your docs here. */
public class Arducam {
    // Just in case, I doubt we may need to use more than one strategy.
    PoseStrategy[] poseStrategies = {
        PoseStrategy.AVERAGE_BEST_TARGETS,
        PoseStrategy.LOWEST_AMBIGUITY,
        PoseStrategy.PNP_DISTANCE_TRIG_SOLVE
    };
    
    PhotonPipelineResult result;
    PhotonCamera m_arduCamera;
    PhotonPipelineResult m_pipelineResult = new PhotonPipelineResult(0, 0, 0, 0, null);
    PhotonPoseEstimator m_photonPoseEstimator = new PhotonPoseEstimator(AprilTagFieldLayout.loadField(AprilTagFields.k2025ReefscapeAndyMark), poseStrategies[0], null);
    public Arducam(String cameraName) {
        m_arduCamera = new PhotonCamera(NetworkTableInstance.getDefault(), cameraName);
        result = m_arduCamera.getLatestResult();
        
    }

}

// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.lib.util;

import java.util.List;

import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.proto.Photon;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

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
        PoseStrategy.PNP_DISTANCE_TRIG_SOLVE,
        PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR
    };
    
    PhotonPipelineResult result;
    PhotonCamera m_arduCamera;
    PhotonPipelineResult m_pipelineResult = new PhotonPipelineResult(0, 0, 0, 0, null);
    PhotonPoseEstimator m_photonPoseEstimator = new PhotonPoseEstimator(AprilTagFieldLayout.loadField(AprilTagFields.k2025ReefscapeWelded), poseStrategies[3], null);
    
    public Arducam(String cameraName) {
        m_arduCamera = new PhotonCamera(NetworkTableInstance.getDefault(), cameraName);
        result = m_arduCamera.getLatestResult();
        List<PhotonTrackedTarget> m_results = result.getTargets();

        m_results.get(0).getPoseAmbiguity();
    }

    /* identified target */
    public boolean hasTarget() {
        boolean hasTarget =  result.hasTargets();
        return hasTarget;
    }

    /* To check if camera actively sends frame */
    public boolean hasConnection() {
        boolean connection = m_arduCamera.isConnected();
        return connection;
    }

    public PhotonTrackedTarget getResultsBest() {
        var best = result.getBestTarget();
        return best;
    }

    public void takeSnapshot() {
        m_arduCamera.takeInputSnapshot();
        m_arduCamera.takeOutputSnapshot();
    }

    



}

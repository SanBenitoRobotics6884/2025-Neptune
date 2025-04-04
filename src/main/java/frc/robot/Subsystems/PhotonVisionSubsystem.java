package frc.robot.Subsystems;

import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class PhotonVisionSubsystem extends SubsystemBase {
    PhotonCamera camera = new PhotonCamera("arducam-6884");
    public PhotonVisionSubsystem() {

    }

    public double getStrafe() {
        PhotonPipelineResult result = camera.getLatestResult();
        if(result.hasTargets()){
            PhotonTrackedTarget target = result.getBestTarget();
            // SmartDashboard.putNumber("PV-yaw", target.getYaw());
            // SmartDashboard.putNumber("PV-pitch", target.getPitch());
            // SmartDashboard.putNumber("PV-area", target.getArea());
            // SmartDashboard.putNumber("PV-skew", target.getSkew());
            // SmartDashboard.putNumber("PV-val", target.fiducialId);
            return -(target.getYaw()+21) / 45;
        }
        return 0.0;
    }
    
    @Override
    public void periodic(){
        PhotonPipelineResult result = camera.getLatestResult();
        if(result.hasTargets()){
            PhotonTrackedTarget target = result.getBestTarget();
            SmartDashboard.putNumber("PV-yaw", target.getYaw());
        }
    }
}
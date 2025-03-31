package frc.robot.Subsystems;

import frc.robot.Constants;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;

import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

//import com.ctre.phoenix.sensors.PigeonIMU;
import com.ctre.phoenix6.configs.Pigeon2Configuration;
import com.ctre.phoenix6.hardware.Pigeon2;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.pathplanner.lib.util.PathPlannerLogging;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;

public class PhotonVisionSubsystem extends SubsystemBase {
    PhotonCamera camera = new PhotonCamera("arducam-6884");
    public PhotonVisionSubsystem() {

    }

    public double getStrafe() {
        PhotonPipelineResult result = camera.getLatestResult();
        if(result.hasTargets()){
            PhotonTrackedTarget target = result.getBestTarget();
            SmartDashboard.putNumber("PV-yaw", target.getYaw());
            SmartDashboard.putNumber("PV-pitch", target.getPitch());
            SmartDashboard.putNumber("PV-area", target.getArea());
            SmartDashboard.putNumber("PV-skew", target.getSkew());
            SmartDashboard.putNumber("PV-val", target.fiducialId);
            return target.getYaw() / 45;
        }
        return 0.0;
    }
    
    @Override
    public void periodic(){
        PhotonPipelineResult result = camera.getLatestResult();
        if(result.hasTargets()){
            PhotonTrackedTarget target = result.getBestTarget();
            SmartDashboard.putNumber("PV-yaw", target.getYaw());
            SmartDashboard.putNumber("PV-pitch", target.getPitch());
            SmartDashboard.putNumber("PV-area", target.getArea());
            SmartDashboard.putNumber("PV-skew", target.getSkew());
            SmartDashboard.putNumber("PV-val", target.fiducialId);
                
        }
    }
}
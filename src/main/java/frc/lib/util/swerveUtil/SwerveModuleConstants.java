package frc.lib.util.swerveUtil;

import edu.wpi.first.math.geometry.Rotation2d;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import frc.robot.Constants;

public class SwerveModuleConstants {
    public final int driveMotorID;
    public final int angleMotorID;
    public final int cancoderID;
    public final Rotation2d angleOffset;
    public final InvertedValue invertMotor;


    /**
     * Swerve Module Constants to be used when creating swerve modules.
     * @param driveMotorID
     * @param angleMotorID
     * @param canCoderID
     * @param angleOffset
     * @param canBus
     */
    public SwerveModuleConstants(int driveMotorID, int angleMotorID, int canCoderID, Rotation2d angleOffset, InvertedValue invertMotor) {
        this.driveMotorID = driveMotorID;
        this.angleMotorID = angleMotorID;
        this.cancoderID = canCoderID;
        this.angleOffset = angleOffset;
        this.invertMotor = invertMotor;
    }

    public TalonFXConfiguration asTalonConfig() {
        TalonFXConfiguration swerveDriveTalonConfig = new TalonFXConfiguration();
        /** Swerve Drive Motor Configuration */
        /* Motor Inverts and Neutral Mode */
        swerveDriveTalonConfig.MotorOutput.Inverted = this.invertMotor;
        //swerveDriveTalonConfig.MotorOutput.NeutralMode = Constants.Swerve.driveNeutralMode;
        swerveDriveTalonConfig.Feedback.SensorToMechanismRatio = Constants.Swerve.driveGearRatio;
        swerveDriveTalonConfig.CurrentLimits.SupplyCurrentLimitEnable = Constants.Swerve.driveEnableCurrentLimit;
        swerveDriveTalonConfig.CurrentLimits.SupplyCurrentLimit = Constants.Swerve.driveCurrentLimit;
        //swerveDriveTalonConfig.CurrentLimits.SupplyCurrentThreshold = Constants.Swerve.driveCurrentThreshold;
        //swerveDriveTalonConfig.CurrentLimits.SupplyTimeThreshold = Constants.Swerve.driveCurrentThresholdTime;
        swerveDriveTalonConfig.Slot0.kP = Constants.Swerve.driveKP;
        swerveDriveTalonConfig.Slot0.kI = Constants.Swerve.driveKI;
        swerveDriveTalonConfig.Slot0.kD = Constants.Swerve.driveKD;
        swerveDriveTalonConfig.OpenLoopRamps.DutyCycleOpenLoopRampPeriod = Constants.Swerve.openLoopRamp;
        swerveDriveTalonConfig.OpenLoopRamps.VoltageOpenLoopRampPeriod = Constants.Swerve.openLoopRamp;
        swerveDriveTalonConfig.ClosedLoopRamps.DutyCycleClosedLoopRampPeriod = Constants.Swerve.closedLoopRamp;
        swerveDriveTalonConfig.ClosedLoopRamps.VoltageClosedLoopRampPeriod = Constants.Swerve.closedLoopRamp;

        return swerveDriveTalonConfig;
    }

}
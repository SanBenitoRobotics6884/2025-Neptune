package frc.lib.util.swerveUtil;

import edu.wpi.first.math.geometry.Rotation2d;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.configs.TalonFXSConfiguration;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.configs.MagnetSensorConfigs;
import com.ctre.phoenix6.signals.SensorDirectionValue;
import com.ctre.phoenix6.hardware.CANcoder;

import com.revrobotics.spark.config.SparkMaxConfig;
import com.ctre.phoenix6.signals.MotorArrangementValue;
import frc.robot.Constants;

public class SwerveModuleConstants {
    public final int driveMotorID;
    public final int angleMotorID;
    public final int cancoderID;
    public final Rotation2d angleOffset;
    public final boolean driveInvert;
    public final boolean steerInvert;
    public final boolean angleMotorInvert;
    public final double magnetOffset;


    /**
     * Swerve Module Constants to be used when creating swerve modules.
     * @param driveMotorID
     * @param angleMotorID
     * @param canCoderID
     * @param angleOffset
     * @param canBus
     */
    public SwerveModuleConstants(int driveMotorID, int angleMotorID, int canCoderID, Rotation2d angleOffset, boolean driveInvert, boolean steerInvert, boolean angleMotorInvert, double magnetOffset) {
        this.driveMotorID = driveMotorID;
        this.angleMotorID = angleMotorID;
        this.cancoderID = canCoderID;
        this.angleOffset = angleOffset;
        this.angleMotorInvert = angleMotorInvert;
        this.driveInvert = driveInvert;
        this.steerInvert = steerInvert;
        this.magnetOffset = magnetOffset;
    }
    //     /** Swerve CANCoder Configuration */
    //    swerveCANcoderConfig.MagnetSensor.SensorDirection = Constants.Swerve.cancoderInvert;

    public SparkMaxConfig asSwerveAngleConfig() {
       //Swerve angle motor config
       //Motor inverts and nuetral modes
       SparkMaxConfig swerveAngleSparkConfig =  new SparkMaxConfig();
       swerveAngleSparkConfig.inverted(this.steerInvert);
       swerveAngleSparkConfig.idleMode(Constants.Swerve.angleNuetralMode);

       //Gear ratio and wrapping config
       swerveAngleSparkConfig.encoder.positionConversionFactor(360/Constants.Swerve.angleGearRatio);
       swerveAngleSparkConfig.encoder.velocityConversionFactor(Constants.Swerve.angleGearRatio / 60);
       swerveAngleSparkConfig.closedLoop.positionWrappingEnabled(true);

       //current limiting
       swerveAngleSparkConfig.smartCurrentLimit(40);
       return swerveAngleSparkConfig;
   }

    public MagnetSensorConfigs asMagnetSensorConfig() {
        MagnetSensorConfigs config = new MagnetSensorConfigs();
        // config.MagnetOffset = this.magnetOffset;
        //config.SensorDirection = this.steerInvert ?
        //    SensorDirectionValue.CounterClockwise_Positive : SensorDirectionValue.Clockwise_Positive;
        return config;
    }

    public TalonFXSConfiguration asTalonSteerConfig() {
        TalonFXSConfiguration swerveSteerTalonConfig = new TalonFXSConfiguration();
        swerveSteerTalonConfig.Commutation.MotorArrangement = MotorArrangementValue.NEO_JST;
        swerveSteerTalonConfig.MotorOutput.Inverted = angleMotorInvert ? InvertedValue.Clockwise_Positive : InvertedValue.CounterClockwise_Positive;
        // toConfigure.primaryPID.kp = 0.1;
        swerveSteerTalonConfig.Slot0.kP = Constants.Swerve.angleKP;
        swerveSteerTalonConfig.Slot0.kI = Constants.Swerve.angleKI;
        swerveSteerTalonConfig.Slot0.kD = Constants.Swerve.angleKD;
        // swerveSteerTalonConfig.Feedback.SensorToMechanismRatio = Constants.Swerve.angleGearRatio;

        swerveSteerTalonConfig.CurrentLimits.SupplyCurrentLimitEnable = Constants.Swerve.angleEnableCurrentLimit;
        swerveSteerTalonConfig.CurrentLimits.SupplyCurrentLimit = Constants.Swerve.angleCurrentLimit;

        swerveSteerTalonConfig.OpenLoopRamps.DutyCycleOpenLoopRampPeriod = Constants.Swerve.openLoopRamp;
        swerveSteerTalonConfig.OpenLoopRamps.VoltageOpenLoopRampPeriod = Constants.Swerve.openLoopRamp;
        swerveSteerTalonConfig.ClosedLoopRamps.DutyCycleClosedLoopRampPeriod = Constants.Swerve.closedLoopRamp;
        swerveSteerTalonConfig.ClosedLoopRamps.VoltageClosedLoopRampPeriod = Constants.Swerve.closedLoopRamp;
        swerveSteerTalonConfig.MotorOutput.NeutralMode = Constants.Swerve.talonAngleNeutralMode;

        // When we try to do continuousWrap, the motors are unresponsive.
        swerveSteerTalonConfig.ClosedLoopGeneral.ContinuousWrap = false;
        return swerveSteerTalonConfig;
    }

    public TalonFXConfiguration asTalonConfig() {
        TalonFXConfiguration swerveDriveTalonConfig = new TalonFXConfiguration();
        /** Swerve Drive Motor Configuration */
        /* Motor Inverts and Neutral Mode */
        swerveDriveTalonConfig.MotorOutput.Inverted = this.driveInvert ? InvertedValue.Clockwise_Positive : InvertedValue.CounterClockwise_Positive;

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
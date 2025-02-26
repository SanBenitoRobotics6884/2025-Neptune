package frc.robot;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.ctre.phoenix6.configs.TalonFXConfiguration;

public final class HardwareConfigs {
  
    public CANcoderConfiguration swerveCANcoderConfig = new CANcoderConfiguration();
    public SparkMaxConfig swerveAngleSparkConfig =  new SparkMaxConfig();
    public SparkMaxConfig swerveDriveSparkConfig =  new SparkMaxConfig();
    public TalonFXConfiguration swerveDriveTalonConfig = new TalonFXConfiguration();
    public TalonFXConfiguration elevatorTalonConfig = new TalonFXConfiguration();

    public HardwareConfigs(){
        /** Swerve Drive Motor Configuration */
        /* Motor Inverts and Neutral Mode */
        swerveDriveTalonConfig.MotorOutput.Inverted = Constants.Swerve.driveMotorInvert;
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

        elevatorTalonConfig.MotorOutput.Inverted = Constants.Elevator.MOTORINVERT;
        // elevatorTalonConfig.MotorOutput.NeutralMode = Constants.Elevator.NEUTRALMODE;
        elevatorTalonConfig.Feedback.SensorToMechanismRatio = Constants.Elevator.GEARRATIO;
        elevatorTalonConfig.CurrentLimits.SupplyCurrentLimitEnable = Constants.Elevator.ENABLECURRENTLIMIT;
        elevatorTalonConfig.CurrentLimits.SupplyCurrentLimit = Constants.Elevator.CURRENTLIMIT;
        elevatorTalonConfig.Slot0.kP = Constants.Elevator.KP;
        elevatorTalonConfig.Slot0.kI = Constants.Elevator.KI;
        elevatorTalonConfig.Slot0.kD = Constants.Elevator.KD;
        elevatorTalonConfig.OpenLoopRamps.DutyCycleOpenLoopRampPeriod = Constants.Elevator.OPENLOOPRAMP;
        elevatorTalonConfig.OpenLoopRamps.VoltageOpenLoopRampPeriod = Constants.Elevator.OPENLOOPRAMP;
        elevatorTalonConfig.ClosedLoopRamps.DutyCycleClosedLoopRampPeriod = Constants.Elevator.CLOSEDLOOPRAMP;
        elevatorTalonConfig.ClosedLoopRamps.VoltageClosedLoopRampPeriod = Constants.Elevator.CLOSEDLOOPRAMP;

        /** Swerve CANCoder Configuration */
       swerveCANcoderConfig.MagnetSensor.SensorDirection = Constants.Swerve.cancoderInvert;

       //Swerve angle motor config
       //Motor inverts and nuetral modes
       swerveAngleSparkConfig.inverted(Constants.Swerve.angleMotorInvert);
       swerveAngleSparkConfig.idleMode(Constants.Swerve.angleNuetralMode);

       //Gear ratio and wrapping config
       swerveAngleSparkConfig.encoder.positionConversionFactor(360/Constants.Swerve.angleGearRatio);
       swerveAngleSparkConfig.encoder.velocityConversionFactor(Constants.Swerve.angleGearRatio / 60);
       swerveAngleSparkConfig.closedLoop.positionWrappingEnabled(true);

       //current limiting
       swerveAngleSparkConfig.smartCurrentLimit(40);

       //PID config
       swerveDriveSparkConfig.closedLoop.p(Constants.Swerve.driveKP);
       swerveDriveSparkConfig.closedLoop.i(Constants.Swerve.driveKI);
       swerveDriveSparkConfig.closedLoop.d(Constants.Swerve.driveKD);

       //Swerve drive motor config
       //Motor inverts and nuetral modes
       //swerveDriveSparkConfig.inverted(Constants.Swerve.driveMotorInvert);
       swerveDriveSparkConfig.idleMode(Constants.Swerve.driveNuetralMode);

       //Gear ratio and wrapping config
       swerveDriveSparkConfig.encoder.positionConversionFactor(Constants.Swerve.driveGearRatio);
       swerveDriveSparkConfig.closedLoop.positionWrappingEnabled(true);

       //current limiting
       swerveDriveSparkConfig.smartCurrentLimit(40);

       //PID config
       swerveAngleSparkConfig.closedLoop.p(Constants.Swerve.angleKP);
       swerveAngleSparkConfig.closedLoop.i(Constants.Swerve.angleKI);
       swerveAngleSparkConfig.closedLoop.d(Constants.Swerve.angleKD);

       swerveAngleSparkConfig.openLoopRampRate(Constants.Swerve.openLoopRamp);
       swerveAngleSparkConfig.closedLoopRampRate(Constants.Swerve.closedLoopRamp);

    }
}
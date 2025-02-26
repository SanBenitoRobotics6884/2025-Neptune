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

    public HardwareConfigs(){
        CurrentLimitsConfigs currentLimits = new CurrentLimitsConfigs();
        //currentLimits.supplyCurrentLimit = 40.0;
        //currentLimits.supplyCurrenThreshold = 40.0;
        //currentLimits.supplyTimeThreshold = 0.0;
        //currentLimits.supplyCurrentLimitEnable = true;
        swerveDriveTalonConfig.CurrentLimits = currentLimits;

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
       swerveDriveSparkConfig.inverted(Constants.Swerve.driveMotorInvert);
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
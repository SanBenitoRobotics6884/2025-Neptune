package frc.robot;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.signals.InvertedValue;

public final class HardwareConfigs {
  
    public CANcoderConfiguration swerveCANcoderConfig = new CANcoderConfiguration();
    public SparkMaxConfig swerveAngleSparkConfig =  new SparkMaxConfig();
    public SparkMaxConfig swerveDriveSparkConfig =  new SparkMaxConfig();
    public TalonFXConfiguration elevatorTalonConfig = new TalonFXConfiguration();

    public HardwareConfigs(){


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

    public TalonFXConfiguration getElevatorConfig(boolean invertMotor) {
        TalonFXConfiguration elevatorTalonConfig = new TalonFXConfiguration();
        elevatorTalonConfig.MotorOutput.Inverted = invertMotor ? InvertedValue.Clockwise_Positive : InvertedValue.CounterClockwise_Positive;
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
        return elevatorTalonConfig;
    }

}
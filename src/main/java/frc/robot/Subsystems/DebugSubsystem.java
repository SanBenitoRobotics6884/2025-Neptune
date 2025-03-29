package frc.robot.Subsystems;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.PositionDutyCycle;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.hardware.TalonFXS;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.MotorArrangementValue;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import frc.lib.math.Conversions;
import frc.lib.util.swerveUtil.SwerveModuleConstants;
import frc.robot.Constants;

import com.ctre.phoenix6.controls.PositionVoltage;

public class DebugSubsystem extends SubsystemBase {
    TalonFX motor;
    double gearRatio = 150.0/7;
    String CANIVOR_BUS = "Galigma";

    public DebugSubsystem() {
        int MotorID = 1;
        motor = new TalonFX(MotorID, CANIVOR_BUS);
        motor.getConfigurator().apply(this.getMotorConfig());

    }

    public void zero(){
        // double newPosition = adjustedAngle * gearRatio;
        SmartDashboard.putNumber("Before Zero", motor.getPosition().getValueAsDouble());
        StatusCode sc = motor.setPosition(0.5, 10);
        SmartDashboard.putNumber("After Zero", motor.getPosition().getValueAsDouble());
        setAngle(0);
    }

    public void setAngle(double rots){
        double targetAngle = rots * gearRatio;
        PositionDutyCycle position = new PositionDutyCycle(targetAngle);
        motor.setControl(position.withSlot(0));
        SmartDashboard.putNumber("Setting Angle", rots);
    }

    public TalonFXConfiguration getMotorConfig(){
        TalonFXConfiguration config = new TalonFXConfiguration();
        // config.Commutation.MotorArrangement = MotorArrangementValue.NEO_JST;
        boolean angleMotorInvert = false;
        config.MotorOutput.Inverted = angleMotorInvert ? InvertedValue.Clockwise_Positive : InvertedValue.CounterClockwise_Positive;
        // toConfigure.primaryPID.kp = 0.1;
        config.Slot0.kP = 0.1;
        config.Slot0.kI = 0;
        config.Slot0.kD = 0;
        // config.Feedback.SensorToMechanismRatio = Constants.Swerve.angleGearRatio;

        config.CurrentLimits.SupplyCurrentLimitEnable = true;
        config.CurrentLimits.SupplyCurrentLimit = 40;

        config.OpenLoopRamps.DutyCycleOpenLoopRampPeriod = 0.2;
        config.OpenLoopRamps.VoltageOpenLoopRampPeriod = 0.2;
        config.ClosedLoopRamps.DutyCycleClosedLoopRampPeriod = 0.2;
        config.ClosedLoopRamps.VoltageClosedLoopRampPeriod = 0.2;
        config.MotorOutput.NeutralMode = NeutralModeValue.Brake;

        // When we try to do continuousWrap, the motors are unresponsive.
        config.ClosedLoopGeneral.ContinuousWrap = false;
        return config;
    }
}


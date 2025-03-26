// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Subsystems;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.ColorSensorV3;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.ctre.phoenix6.configs.TalonFXSConfiguration;
import com.ctre.phoenix6.hardware.TalonFXS;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.MotorArrangementValue;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

import static frc.robot.Constants.CoralOutIntake.*;

public class CoralOutIntakeSubsystem extends SubsystemBase {
  public TalonFXS m_topMotor = new TalonFXS(TOP_MOTOR_ID);
  public TalonFXS m_bottomMotor = new TalonFXS(BOTTOM_MOTOR_ID);
  String CANIVOR_BUS = "Galigma";

  // UNUSED FOR NOW 3/17
  // ColorSensorV3 m_colorSensor = new ColorSensorV3(null);
  boolean topMotorInvert = false;
  boolean bottomMotorInvert = false;
  double MOTOR_SPEED = 0.5 * (1.0/3);
  int STALL_CURRENT_THRESHOLD = 15; // amps

  /** Creates a new CoralIntakeOuttake. */
  public CoralOutIntakeSubsystem() {
    m_topMotor = new TalonFXS(TOP_MOTOR_ID, CANIVOR_BUS);
    m_topMotor.getConfigurator().apply(getTalonFXSConfiguration(topMotorInvert));

    m_bottomMotor = new TalonFXS(BOTTOM_MOTOR_ID, CANIVOR_BUS);
    m_bottomMotor.getConfigurator().apply(getTalonFXSConfiguration(bottomMotorInvert));
  }

  public TalonFXSConfiguration getTalonFXSConfiguration(boolean invertMotor) {
        TalonFXSConfiguration talonFXSConfig = new TalonFXSConfiguration();
        talonFXSConfig.Commutation.MotorArrangement = MotorArrangementValue.NEO550_JST;
        talonFXSConfig.MotorOutput.Inverted = invertMotor ? InvertedValue.Clockwise_Positive : InvertedValue.CounterClockwise_Positive;
        // toConfigure.primaryPID.kp = 0.1;
        talonFXSConfig.Slot0.kP = Constants.Swerve.angleKP;
        talonFXSConfig.Slot0.kI = Constants.Swerve.angleKI;
        talonFXSConfig.Slot0.kD = Constants.Swerve.angleKD;
        // talonFXSConfig.Feedback.SensorToMechanismRatio = Constants.Swerve.angleGearRatio;

        talonFXSConfig.CurrentLimits.SupplyCurrentLimitEnable = Constants.Swerve.angleEnableCurrentLimit;
        talonFXSConfig.CurrentLimits.SupplyCurrentLimit = Constants.Swerve.angleCurrentLimit;

        talonFXSConfig.OpenLoopRamps.DutyCycleOpenLoopRampPeriod = Constants.Swerve.openLoopRamp;
        talonFXSConfig.OpenLoopRamps.VoltageOpenLoopRampPeriod = Constants.Swerve.openLoopRamp;
        talonFXSConfig.ClosedLoopRamps.DutyCycleClosedLoopRampPeriod = Constants.Swerve.closedLoopRamp;
        talonFXSConfig.ClosedLoopRamps.VoltageClosedLoopRampPeriod = Constants.Swerve.closedLoopRamp;
        talonFXSConfig.MotorOutput.NeutralMode = Constants.Swerve.talonAngleNeutralMode;

        // When we try to do continuousWrap, the motors are unresponsive.
        talonFXSConfig.ClosedLoopGeneral.ContinuousWrap = false;
        return talonFXSConfig;

  }

  public boolean pieceIsIn() {
    // Update function to return results of Rev Color Sensor v3
    return false; // m_topMotor.getAppliedOutput() > STALL_CURRENT_THRESHOLD || m_bottomMotor.getAppliedOutput() > STALL_CURRENT_THRESHOLD;
  }

  public void intakeOuttake(Boolean intakePressed, Boolean outtakePressed) {
    if (intakePressed) {
      m_topMotor.setVoltage(MOTOR_SPEED);
      m_bottomMotor.setVoltage(MOTOR_SPEED);
    } else if (outtakePressed) {
      m_topMotor.setVoltage(-MOTOR_SPEED);
      m_bottomMotor.setVoltage(-MOTOR_SPEED);
    } else {
      m_topMotor.setVoltage(0);
      m_bottomMotor.setVoltage(0);
    }
  }

  // Needs an outtake function using x button

  /*
  public Command toHighPosotion() {
    return runOnce(() -> setSetpoint(HIGH_POSITION));
  }

  public Command toMidPosotion() {
    return runOnce(() -> setSetpoint(MID_POSITION));
  }

  public Command toLowPosotion() {
    return runOnce(() -> setSetpoint(LOW_POSITION));
  }
  */

  @Override
  public void periodic() {

    // This method will be called once per scheduler run
    // Problem m_pivotEncoder is null
    // m_PID.setSetpoint(m_pivotSetpoint);
    // m_pivotMotor.set(m_PID.calculate(m_pivotEncoder.getPosition()));

  }
}

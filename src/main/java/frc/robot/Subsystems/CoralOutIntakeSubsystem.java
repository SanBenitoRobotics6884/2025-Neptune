// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Subsystems;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import static frc.robot.Constants.CoralOutIntake.*;

public class CoralOutIntakeSubsystem extends SubsystemBase {
  Joystick m_joystick = new Joystick(JOYSTICK_PORT_ID);

  SparkMax m_pivotMotor = new SparkMax(PIVOTion_MOTOR_ID, MotorType.kBrushless);
  SparkMax m_stealOrNoStealMotor = new SparkMax(ROTATION_MOTOR_ID, MotorType.kBrushless);
  SparkMaxConfig config = new SparkMaxConfig();

  AbsoluteEncoder m_pivotEncoder;
  AbsoluteEncoder m_rotationEncoder;

  DigitalInput m_limitSwitch = new DigitalInput(LIMITSWITCH_CHANNEL_ID);

  double m_pivotSetpoint;

  PIDController m_PID = new PIDController(Kp, Ki, Kd);

  /** Creates a new CoralIntakeOuttake. */
  public CoralOutIntakeSubsystem() {
    config.inverted(true);
    m_pivotMotor.configure(config, null, null);
    m_stealOrNoStealMotor.configure(config, null, null);
  }

  boolean pieceIsIn() {
    return m_limitSwitch.get();
  }

  public void setSetpoint(double setpoint) {
    m_pivotSetpoint = setpoint;
  }

  public void intakeOutaking() {
    if (pieceIsIn()) {
      m_stealOrNoStealMotor.set(-ROTATION_MOTOR_SPEED);
    } else {
      m_stealOrNoStealMotor.set(ROTATION_MOTOR_SPEED);
    }
  }

  public Command toHighPosotion() {
    return runOnce(() -> setSetpoint(HIGH_POSITION));
  }

  public Command toMidPosotion() {
    return runOnce(() -> setSetpoint(MID_POSITION));
  }

  public Command toLowPosotion() {
    return runOnce(() -> setSetpoint(LOW_POSITION));
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    // Problem m_pivotEncoder is null
    // m_PID.setSetpoint(m_pivotSetpoint);
    // m_pivotMotor.set(m_PID.calculate(m_pivotEncoder.getPosition()));
  }
}

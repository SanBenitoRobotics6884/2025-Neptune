// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Subsystems;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.ColorSensorV3;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import static frc.robot.Constants.CoralOutIntake.*;

public class CoralOutIntakeSubsystem extends SubsystemBase {
  SparkMax m_topMotor = new SparkMax(TOP_MOTOR_ID, MotorType.kBrushless);
  SparkMax m_bottomMotor = new SparkMax(BOTTOM_MOTOR_ID, MotorType.kBrushless);
  SparkMaxConfig config = new SparkMaxConfig();
  SparkMaxConfig config1 = new SparkMaxConfig();
  // UNUSED FOR NOW 3/17
  // ColorSensorV3 m_colorSensor = new ColorSensorV3(null);
  double MOTOR_SPEED = 0.5 * (1/3);
  int STALL_CURRENT_THRESHOLD = 15; // amps

  /** Creates a new CoralIntakeOuttake. */
  public CoralOutIntakeSubsystem() {
    config.inverted(true);
    config1.inverted(false);
    // m_pivotMotor.configure(config, null, null);
    m_topMotor.configure(config, null, null);
    m_bottomMotor.configure(config1, null, null);

  }

  public boolean pieceIsIn() {
    // Update function to return results of Rev Color Sensor v3
    return m_topMotor.getAppliedOutput() > STALL_CURRENT_THRESHOLD || m_bottomMotor.getAppliedOutput() > STALL_CURRENT_THRESHOLD;
  }

  public void intakeOuttake(Boolean intakePressed, Boolean outtakePressed) {
    if (intakePressed) {
      m_topMotor.set(MOTOR_SPEED);
      m_bottomMotor.set(MOTOR_SPEED);
    } else if (outtakePressed) {
      m_topMotor.set(-MOTOR_SPEED);
      m_bottomMotor.set(-MOTOR_SPEED);
    } else {
      m_topMotor.stopMotor();
      m_bottomMotor.stopMotor();
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

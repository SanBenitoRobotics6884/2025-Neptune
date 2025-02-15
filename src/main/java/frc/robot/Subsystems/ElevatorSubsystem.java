// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Subsystems;

import com.ctre.phoenix6.hardware.TalonFX;
import com.revrobotics.AbsoluteEncoder;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import static frc.robot.Constants.Elevator.*;

public class ElevatorSubsystem extends SubsystemBase {
  private TalonFX m_motor1 = new TalonFX(MOTOR_1_ID);
  // private TalonFX m_moter2;

  private AbsoluteEncoder relativeEncoder_1;
  // private RelativeEncoder relativeEncodor_2;

  public double m_setpoint;

  public XboxController xboxController;
  private PIDController pidController;

  public ElevatorSubsystem() {
    pidController = new PIDController(Kp, Ki, Kd);
    pidController.setTolerance(ERROR_TOLERANCE);
    m_motor1.set(pidController.calculate(relativeEncoder_1.getPosition(),m_setpoint));
    
    /** if (xboxController.getAButtonPressed()) {
      setpoint = 1;
    }

    if (xboxController.getBButtonPressed()) {
      setpoint = 2;
    }

    if (xboxController.getYButtonPressed()) {
      setpoint = 3;
    }

    if (xboxController.getLeftBumperButtonPressed()) {
      setpoint = 4;
    }

    if (xboxController.getRightBumperButtonPressed()) {
      setpoint = 5;
    } */
  }

  public void setSetpoit(double setpoint) {
    m_setpoint = setpoint;
  }

  @Override
  public void periodic() {
    pidController.atSetpoint();
  }
}

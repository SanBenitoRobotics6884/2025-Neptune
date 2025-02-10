// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Subsystems;

import com.ctre.phoenix6.hardware.TalonFX;
import com.revrobotics.RelativeEncoder;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;


public class ElevatorSubsystem extends SubsystemBase {
  public TalonFX m_motor1;
 // private TalonFX m_moter2;
  private RelativeEncoder relativeEncoder_1;
  // private RelativeEncoder relativeEncoder_2;
  public double setpoint;
  public XboxController xboxController;
  private PIDController pidController;
  public ElevatorSubsystem() {
    pidController = new PIDController(0.5, 0.01, 0);
    pidController.setTolerance(0.2);
    m_motor1.set(pidController.calculate(relativeEncoder_1.getPosition(),setpoint));

    if (xboxController.getAButtonPressed()) {
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


      




    }









    }

 


  public void periodic() {
  pidController.atSetpoint();
  }
}

// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Subsystems;

import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.hardware.TalonFX;
import com.revrobotics.AbsoluteEncoder;

import static edu.wpi.first.units.Units.Degrees;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.motorcontrol.Talon;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.math.Conversions;
import frc.robot.Constants;
import frc.robot.HardwareConfigs;

import static frc.robot.Constants.Elevator.*;

public class ElevatorSubsystem extends SubsystemBase {
 /* Left and Right Motors */
  private TalonFX m_leftMotor;
  private TalonFX m_rightMotor;

  private PIDController pidController;
  private Follower m_follower = new Follower(LEFT_MOTOR_ID, true);
  private double speed = 0.5;
  private double THRESHOLD = 5.0;
  private double LOWER_LIMIT = 0.0;
  private double UPPPER_LIMIT = 3.0;
  private double lastAction = System.currentTimeMillis();

  double eMotorPosition = 0.0;

  public ElevatorSubsystem() {

    HardwareConfigs hardwareConfigs = new HardwareConfigs();
    m_leftMotor = new TalonFX(LEFT_MOTOR_ID, "Galigma");
    m_rightMotor = new TalonFX(RIGHT_MOTOR_ID, "Galigma");
    // Load your TalonFX drive config (PID, current limit, etc.)
    var m_leftMotorConfigurator = m_leftMotor.getConfigurator();
    //var m_rightMotorConfigurator = m_rightMotor.getConfigurator();
    m_leftMotorConfigurator.apply(hardwareConfigs.getElevatorConfig(false));
    //m_rightMotorConfigurator.apply(hardwareConfigs.getElevatorConfig(true));
    m_leftMotorConfigurator.setPosition(0.0);
    //m_rightMotorConfigurator.setPosition(0.0);
    m_rightMotor.setControl(m_follower);

    eMotorPosition = 0.0;
  }

  public void extend(double val){
    if (System.currentTimeMillis() - lastAction < 100){ // every 100ms
      return;
    }
    lastAction = System.currentTimeMillis();
    eMotorPosition += val;
    if (eMotorPosition > UPPPER_LIMIT) {
      eMotorPosition = UPPPER_LIMIT;
    }

    m_leftMotor.set(speed);
    // m_rightMotor.set(-speed);
  }

  public void retract(double val){
    if (System.currentTimeMillis() - lastAction < 100){ // every 100ms
      return;
    }
    lastAction = System.currentTimeMillis();
    eMotorPosition -= val;
    if (eMotorPosition < LOWER_LIMIT) {
      eMotorPosition = LOWER_LIMIT;
    }

    m_leftMotor.set(-speed);
    //m_rightMotor.set(speed);
  }

  public void stop(){
    eMotorPosition = m_leftMotor.getPosition().getValue().in(Degrees);
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("Elevator actual", m_leftMotor.getPosition().getValue().in(Degrees));
    SmartDashboard.putNumber("Elevator target", eMotorPosition);
    if(Math.abs(eMotorPosition - m_leftMotor.getPosition().getValue().in(Degrees)) > THRESHOLD)
      m_leftMotor.setPosition(Conversions.degreesToRotations(eMotorPosition));
  }
}

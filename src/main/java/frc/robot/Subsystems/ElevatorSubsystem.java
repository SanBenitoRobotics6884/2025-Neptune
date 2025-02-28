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

    // pidController = new PIDController(KP, KI, KD);
    // pidController.setTolerance(ERROR_TOLERANCE);
    // mMotor.set(pidController.calculate(relativeEncoder_1.getPosition(), m_setpoint));
  }

  public void extend(double val){
    m_leftMotor.setPosition(0);
    m_leftMotor.set(val);
    m_rightMotor.setPosition(0);
    m_rightMotor.set(val);
  }

  public void retract(double val){
    m_leftMotor.setPosition(0);
    m_leftMotor.set(-val);
    m_rightMotor.setPosition(0);
    m_rightMotor.set(-val);
  }

  public void stop(){
    m_leftMotor.stopMotor();
    m_rightMotor.stopMotor();
  }

  public void setSetpoint(int degrees) {
    m_leftMotor.set(Conversions.degreesToFalcon(degrees, Constants.Elevator.GEARRATIO));
    m_rightMotor.set(Conversions.degreesToFalcon(degrees, Constants.Elevator.GEARRATIO));
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("Elevator Left ", m_leftMotor.getPosition().getValue().in(Degrees));
    SmartDashboard.putNumber("Elevator Right ", m_rightMotor.getPosition().getValue().in(Degrees));
    //pidController.atSetpoint();
    // if (xboxController.getLeftBumperButtonPressed()) {
    //   setSetpoit(0);
    // }
    // if (xboxController.getRightBumperButtonPressed()) {
    //   setSetpoit(90); //TODO: change this to the correct angle
    // }
  }
}

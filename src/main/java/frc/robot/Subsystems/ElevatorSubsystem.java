// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Subsystems;

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

  private TalonFX mLeftMotor;
  private TalonFX mRightMotor;
  // private TalonFX m_moter2;

  // private AbsoluteEncoder relativeEncoder_1;
  // private RelativeEncoder relativeEncodor_2;

  public double m_setpoint;

  public XboxController xboxController;
  private PIDController pidController;

  public ElevatorSubsystem() {

    HardwareConfigs hardwareConfigs = new HardwareConfigs();
    mLeftMotor = new TalonFX(LEFT_MOTOR_ID, "Galigma");
    mRightMotor = new TalonFX(RIGHT_MOTOR_ID, "Galigma");
    // Load your TalonFX drive config (PID, current limit, etc.)
    mLeftMotor.getConfigurator().apply(hardwareConfigs.getElevatorConfig(false));
    mRightMotor.getConfigurator().apply(hardwareConfigs.getElevatorConfig(true));
    mLeftMotor.getConfigurator().setPosition(0.0);
    mRightMotor.getConfigurator().setPosition(0.0);

    // pidController = new PIDController(KP, KI, KD);
    // pidController.setTolerance(ERROR_TOLERANCE);
    // mMotor.set(pidController.calculate(relativeEncoder_1.getPosition(), m_setpoint));
  }

  public void extend(double val){
    mLeftMotor.setPosition(0);
    mLeftMotor.set(val);
    mRightMotor.setPosition(0);
    mRightMotor.set(val);
  }

  public void retract(double val){
    mLeftMotor.setPosition(0);
    mLeftMotor.set(-val);
    mRightMotor.setPosition(0);
    mRightMotor.set(-val);
  }

  public void stop(){
    mLeftMotor.stopMotor();
    mRightMotor.stopMotor();
  }

  public void setSetpoint(int degrees) {
    mLeftMotor.set(Conversions.degreesToFalcon(degrees, Constants.Elevator.GEARRATIO));
    mRightMotor.set(Conversions.degreesToFalcon(degrees, Constants.Elevator.GEARRATIO));
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("Elevator Left ", mLeftMotor.getPosition().getValue().in(Degrees));
    SmartDashboard.putNumber("Elevator Right ", mRightMotor.getPosition().getValue().in(Degrees));
    // pidController.atSetpoint();
    // if (xboxController.getLeftBumperButtonPressed()) {
    //   setSetpoit(0);
    // }
    // if (xboxController.getRightBumperButtonPressed()) {
    //   setSetpoit(90); //TODO: change this to the correct angle
    // }
  }
}

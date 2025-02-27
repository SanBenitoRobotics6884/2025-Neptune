// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Subsystems;

import com.ctre.phoenix6.hardware.TalonFX;
import com.revrobotics.AbsoluteEncoder;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.math.Conversions;
import frc.robot.Constants;
import frc.robot.HardwareConfigs;

import static frc.robot.Constants.Elevator.*;

public class ElevatorSubsystem extends SubsystemBase {

  private TalonFX mMotor;
  // private TalonFX m_moter2;

  // private AbsoluteEncoder relativeEncoder_1;
  // private RelativeEncoder relativeEncodor_2;

  public double m_setpoint;

  public XboxController xboxController;
  private PIDController pidController;

  public ElevatorSubsystem() {
    HardwareConfigs hardwareConfigs = new HardwareConfigs();
    mMotor = new TalonFX(MOTOR_ID, "Galigma");
    // Load your TalonFX drive config (PID, current limit, etc.)
    mMotor.getConfigurator().apply(hardwareConfigs.elevatorTalonConfig);
    mMotor.getConfigurator().setPosition(0.0);

    // pidController = new PIDController(KP, KI, KD);
    // pidController.setTolerance(ERROR_TOLERANCE);
    // mMotor.set(pidController.calculate(relativeEncoder_1.getPosition(), m_setpoint));
  }

  public void setSetpoit(int degrees) {
    mMotor.set(Conversions.degreesToFalcon(degrees, Constants.Elevator.GEARRATIO));
  }

  @Override
  public void periodic() {
    pidController.atSetpoint();
    if (xboxController.getLeftBumperButtonPressed()) {
      setSetpoit(0);
    }
    if (xboxController.getRightBumperButtonPressed()) {
      setSetpoit(90); //TODO: change this to the correct angle
    }
  }
}

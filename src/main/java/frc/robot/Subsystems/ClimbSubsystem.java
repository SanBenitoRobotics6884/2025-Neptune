// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Subsystems;

import static edu.wpi.first.units.Units.Degrees;

import com.ctre.phoenix6.configs.TalonFXConfigurator;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class ClimbSubsystem extends SubsystemBase {
  public TalonFX m_climbMotor;
  TalonFXConfigurator configurator;
  Encoder encoder;
  PIDController pidController;
  /** Creates a new ClimbSubsystem. */
  public ClimbSubsystem() {
    m_climbMotor = new TalonFX(15, "Galigma");
    m_climbMotor.getConfigurator().setPosition(0.0);
    m_climbMotor.optimizeBusUtilization();
    // encoder = new Encoder(0, 0);
    // encoder.setDistancePerPulse(1.0);
    // pidController = new PIDController(0, 0, 0);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("Climb Motor CAN ", m_climbMotor.getPosition().getValue().in(Degrees));
  }
}

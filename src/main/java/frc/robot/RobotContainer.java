// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.Commands.CoralOutIntakeCommand;
import frc.robot.Subsystems.CoralOutIntakeSybsystem;

public class RobotContainer {
  private final CoralOutIntakeSybsystem m_coralOutIntakeSybsystem = new CoralOutIntakeSybsystem();
  private final CoralOutIntakeCommand m_CoralOutIntakeCommand = new CoralOutIntakeCommand(m_coralOutIntakeSybsystem);

  private final Joystick m_joystick = new Joystick(0);

  public RobotContainer() {
    configureBindings();
  }

  private void configureBindings() {
    new JoystickButton(m_joystick, 1)
      .whileTrue(m_CoralOutIntakeCommand);

    new JoystickButton(m_joystick, 10)
      .onTrue(m_coralOutIntakeSybsystem.toHighPosotion());

    new JoystickButton(m_joystick, 11)
      .onTrue(m_coralOutIntakeSybsystem.toMidPosotion());

    new JoystickButton(m_joystick, 12)
      .onTrue(m_coralOutIntakeSybsystem.toLowPosotion());
  }

  public Command getAutonomousCommand() {
    return Commands.print("No autonomous command configured");
  }
}

// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;

public class RobotContainer {
  
  XboxController m_XboxController = new XboxController(0);
ClimbCommand m_ClimbCommand = new ClimbCommand();
ClimbCommandUp m_ClimbCommandUp = new ClimbCommandUp();
ClimbStop m_ClimbStop = new ClimbStop();
  public RobotContainer() {
    configureBindings();
if(m_XboxController.getLeftBumperButtonPressed()){
  m_ClimbCommand.schedule();
}
if (m_XboxController.getAButtonPressed()) {
  m_ClimbStop.schedule();
}
if (m_XboxController.getLeftBumperButtonReleased()) {
  m_ClimbCommandUp.schedule();
}

  }

  private void configureBindings() {}

  public Command getAutonomousCommand() {
    return Commands.print("No autonomous command configured");
  }
}

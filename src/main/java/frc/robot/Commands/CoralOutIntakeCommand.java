// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Commands;

import java.util.function.BooleanSupplier;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Subsystems.CoralOutIntakeSubsystem;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class CoralOutIntakeCommand extends Command {
  CoralOutIntakeSubsystem m_coralOutIntake;

  BooleanSupplier m_xButtonPressed;
  BooleanSupplier m_xButtonReleased;
  BooleanSupplier m_bButtonPressed;
  BooleanSupplier m_bButtonReleased;


  /** Creates a new CoralOutIntakeCommand. */
  public CoralOutIntakeCommand(CoralOutIntakeSubsystem coralOutIntake, BooleanSupplier xButtonPressed, 
  BooleanSupplier xButtonReleased, BooleanSupplier bButtonPressed, BooleanSupplier bButtonReleased) {
    m_coralOutIntake = coralOutIntake;
    addRequirements(coralOutIntake);

    m_xButtonPressed = xButtonPressed;
    m_xButtonReleased = xButtonReleased;
    m_bButtonPressed = bButtonPressed;
    m_bButtonReleased = bButtonReleased;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    Boolean bButtonIsPressed = m_bButtonPressed.getAsBoolean();
    Boolean bButtonIsReleased = m_bButtonPressed.getAsBoolean();

    m_coralOutIntake.intake(bButtonIsPressed, bButtonIsReleased);
  }


  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}

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

  BooleanSupplier m_intakeButton;
  BooleanSupplier m_outtakeButton;
  BooleanSupplier m_pivotUpButton;
  BooleanSupplier m_pivotDownButton;

  /** Creates a new CoralOutIntakeCommand. */
  public CoralOutIntakeCommand(CoralOutIntakeSubsystem coralOutIntake, BooleanSupplier intakeButton, BooleanSupplier outtakeButton) {
    m_coralOutIntake = coralOutIntake;
    addRequirements(coralOutIntake);
    m_intakeButton = intakeButton;
    m_outtakeButton = outtakeButton;
    //m_pivotUpButton = pivotUpButton;
    //m_pivotDownButton = pivotDownButton;

  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_coralOutIntake.intakeOuttake(
      m_intakeButton.getAsBoolean(),
      m_outtakeButton.getAsBoolean()
    );
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

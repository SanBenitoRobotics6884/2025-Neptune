// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;

import frc.robot.commands.*;
import frc.robot.subsystems.*;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
    /* Controllers */
    private final Joystick driver = new Joystick(0);
    ClimbCommand m_ClimbCommand = new ClimbCommand();
    ClimbCommandUp m_ClimbCommandUp = new ClimbCommandUp();
    private final CoralOutIntakeSybsystem m_coralOutIntakeSybsystem = new CoralOutIntakeSybsystem();
    private final CoralOutIntakeCommand m_CoralOutIntakeCommand = new CoralOutIntakeCommand(m_coralOutIntakeSybsystem);

    ClimbStop m_ClimbStop = new ClimbStop();

   /* Driver Controls */
	private final int translationAxis = 1;
	private final int strafeAxis = 0;
	private final int rotationAxis = 4;

    /* Driver Buttons */
    private final JoystickButton zeroGyro = new JoystickButton(driver, XboxController.Button.kY.value);
    private final JoystickButton robotCentric = new JoystickButton(driver, XboxController.Button.kLeftBumper.value);

    private final JoystickButton dampen = new JoystickButton(driver, XboxController.Button.kRightBumper.value);

    private final JoystickButton DynamicLock = new JoystickButton(driver, XboxController.Button.kX.value);

    private final Trigger forwardHold = new Trigger(() -> (driver.getRawAxis(4) > 0.75));
    private final Trigger backwardHold = new Trigger(() -> (driver.getRawAxis(4) < -0.75));

    /* Subsystems */
    private final PoseEstimator s_PoseEstimator = new PoseEstimator();
    private final Swerve s_Swerve = new Swerve(s_PoseEstimator);
    //private final Vision s_Vision = new Vision(s_PoseEstimator);

    /* AutoChooser */
    private final SendableChooser<Command> autoChooser;



    /** The container for the robot. Contains subsystems, OI devices, and commands. */
    public RobotContainer() {
        s_Swerve.setDefaultCommand(
            new SwerveCommand(
                s_Swerve, 
                () -> -driver.getRawAxis(translationAxis), 
                () -> -driver.getRawAxis(strafeAxis), 
                () -> -driver.getRawAxis(rotationAxis), 
                () -> robotCentric.getAsBoolean(),
                () -> dampen.getAsBoolean(),
                () -> 0 // Dynamic heading placeholder
            )
        );

        // Configure the button bindings
        configureButtonBindings();

        if(m_XboxController.getLeftBumperButtonPressed()){
          m_ClimbCommand.schedule();
        }
        if (m_XboxController.getAButtonPressed()) {
          m_ClimbStop.schedule();
        }
        if (m_XboxController.getLeftBumperButtonReleased()) {
          m_ClimbCommandUp.schedule();
        }


        //Pathplanner commands - templates
        
        NamedCommands.registerCommand("marker1", Commands.print("Passed marker 1"));
        NamedCommands.registerCommand("marker2", Commands.print("Passed marker 2"));
        NamedCommands.registerCommand("print hello", Commands.print("hello"));
    
        
        //Auto chooser
        autoChooser = AutoBuilder.buildAutoChooser("New Auto"); // Default auto will be `Commands.none()`
        SmartDashboard.putData("Auto Mode", autoChooser);
    }

    /**
     * Use this method to define your button->command mappings. Buttons can be created by
     * instantiating a {@link GenericHID} or one of its subclasses ({@link
     * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
     * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
     */
    private void configureButtonBindings() {
        /* Driver Buttons */
        zeroGyro.onTrue(new InstantCommand(() -> s_Swerve.zeroHeading()));

    //Heading lock bindings
        forwardHold.onTrue(
            new InstantCommand(() -> States.driveState = States.DriveStates.forwardHold)).onFalse(
            new InstantCommand(() -> States.driveState = States.DriveStates.standard)
        );
        backwardHold.onTrue(
            new InstantCommand(() -> States.driveState = States.DriveStates.backwardHold)).onFalse(
            new InstantCommand(() -> States.driveState = States.DriveStates.standard)
        );
        DynamicLock.onTrue(
            new InstantCommand(() -> States.driveState = States.DriveStates.DynamicLock)).onFalse(
            new InstantCommand(() -> States.driveState = States.DriveStates.standard)
        );
    } 

    /**
     * Use this to pass the autonomous command to the main {@link Robot} class.
     *
     * @return the command to run in autonomous
     */
    public Command getAutonomousCommand() {
        // An ExampleCommand will run in autonomous
        return autoChooser.getSelected();
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

// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;

import frc.robot.Commands.*;
import frc.robot.Subsystems.*;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
    /* Controllers */
    private final XboxController operator = new XboxController(0);
      //climb, elevator, and coral (control is blutooth)
    private final Joystick driver = new Joystick(1);
      // Controller in port 1 is driving (controll is cable)
    //private final CoralOutIntakeSybsystem m_coralOutIntakeSybsystem = new CoralOutIntakeSybsystem();
    //private final CoralOutIntakeCommand m_CoralOutIntakeCommand = new CoralOutIntakeCommand(m_coralOutIntakeSybsystem);

   /* Driver Controls */
	private final int translationAxis = 1;
	private final int strafeAxis = 0;
    private final int triggerLeft = 4; //was 2
    private final int triggerRight = 3; //was 3
	private final int rotationAxis = 4;

    private final int aButton = 1;
    private final int bButton = 2;
    private final int xButton = 4;
    private final int yButton = 5;
    private final int dLeftShoulderButton = 7;

    /* Operator Controls */
    private final int oAButton = 0;
    private final int oBButton = 1;
    private final int oXButton = 2;
    private final int oYButton = 3;
    private final int oLeftShoulderButton = 7;
    private final int oRightShoulderButton = 8;

    /* Driver/Operator Buttons */
    private final JoystickButton climbDisengage = new JoystickButton(operator, oLeftShoulderButton);
    private final JoystickButton climbEngage = new JoystickButton(operator, oRightShoulderButton);

    private final JoystickButton zeroGyro = new JoystickButton(driver, yButton);
    private final JoystickButton dampen = new JoystickButton(driver, dLeftShoulderButton);

    /* Subsystems */
    private final PoseEstimator s_PoseEstimator = new PoseEstimator();
    private final Swerve s_Swerve = new Swerve(s_PoseEstimator);
    private final ClimbSubsystem s_ClimbSubsystem = new ClimbSubsystem();
    private final CoralOutIntakeSubsystem s_CoralOutIntakeSubsystem = new CoralOutIntakeSubsystem();
    private final ElevatorSubsystem s_ElevatorSubsystem = new ElevatorSubsystem();
    // private final Camera s_Camera = new Camera();
    //private final Vision s_Vision = new Vision(s_PoseEstimator);

    /* AutoChooser */
    //private final SendableChooser<Command> autoChooser;

    /** The container for the robot. Contains subsystems, OI devices, and commands. */
    public RobotContainer() {
        s_Swerve.setDefaultCommand(
            new SwerveCommand(
                s_Swerve, 
                () -> driver.getRawAxis(translationAxis),
                () -> driver.getRawAxis(strafeAxis),
                () -> -driver.getRawAxis(rotationAxis), 
                () -> false,
                () -> dampen.getAsBoolean(),
                () -> 0, // Dynamic heading placeholder,
                () -> zeroGyro.getAsBoolean()
            )
        );
/*
        s_ClimbSubsystem.setDefaultCommand(
            new ClimbCommand(
              s_ClimbSubsystem, 
<<<<<<< HEAD
              ()->climbDisengage.getAsBoolean(), 
              ()->climbEngage.getAsBoolean()
            )
        );

        s_CoralOutIntakeSubsystem.setDefaultCommand(
            new CoralOutIntakeCommand(
              s_CoralOutIntakeSubsystem,
              () -> operator.getXButtonPressed(),
              () -> operator.getXButtonReleased(),
              () -> operator.getBButtonPressed(),
              () -> operator.getBButtonReleased()
              // ()->climbDisengage.getAsBoolean(), 
              // ()->climbEngage.getAsBoolean()
=======
              ()->climbDisengage.getAsBoolean(),
              ()->climbEngage.getAsBoolean()
>>>>>>> 26d231c563b6b1cf33fd75f06582c2d8dcb520b6
            )
        );
*/
        // s_CoralOutIntakeSubsystem.setDefaultCommand(
        //     new CoralOutIntakeCommand(
        //       s_CoralOutIntakeSubsystem,
        //       () -> operator.getXButtonPressed(),
        //       () -> operator.getYButtonPressed(),
        //       () -> operator.getAButtonPressed(),
        //       () -> operator.getBButtonPressed()
        //     )
        // );

        s_ElevatorSubsystem.setDefaultCommand(
             new ElevatorCommand(
                 s_ElevatorSubsystem,
                 () -> operator.getLeftY(),
                 () -> operator.getRightY(),
                 () -> operator.getLeftBumper(),
                 () -> operator.getAButtonPressed()
            )
         );

        // Configure the button bindings
        configureButtonBindings();

        //Auto chooser
        //autoChooser = AutoBuilder.buildAutoChooser("New Auto"); // Default auto will be `Commands.none()`
        //SmartDashboard.putData("Auto Mode", autoChooser);
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
      /**
        // forwardHold.onTrue(
        //     new InstantCommand(() -> States.driveState = States.DriveStates.forwardHold)).onFalse(
        //     new InstantCommand(() -> States.driveState = States.DriveStates.standard)
        // );
        // backwardHold.onTrue(
        //     new InstantCommand(() -> States.driveState = States.DriveStates.backwardHold)).onFalse(
        //     new InstantCommand(() -> States.driveState = States.DriveStates.standard)
        // );
        // DynamicLock.onTrue(
        //     new InstantCommand(() -> States.driveState = States.DriveStates.DynamicLock)).onFalse(
        //     new InstantCommand(() -> States.driveState = States.DriveStates.standard)
        // );
      */
    } 

    /**
     * Use this to pass the autonomous command to the main {@link Robot} class.
     *
     * @return the command to run in autonomous
     */

     //Below is Autocommand as of 3:05 pm,, 3/1/2025
    public Command getAutonomousCommand() {
        // An ExampleCommand will run in autonomous
        //return autoChooser.getSelected();
      return new AutonomousCommand(s_Swerve);
    }

  private void configureBindings() {
    /*new JoystickButton(driver, 1)
      .whileTrue(m_CoralOutIntakeCommand);
    new JoystickButton(driver, 10)
      .onTrue(m_coralOutIntakeSybsystem.toHighPosotion());
    new JoystickButton(driver, 11)
      .onTrue(m_coralOutIntakeSybsystem.toMidPosotion());
    new JoystickButton(driver, 12)
      .onTrue(m_coralOutIntakeSybsystem.toLowPosotion());*/
  }
}

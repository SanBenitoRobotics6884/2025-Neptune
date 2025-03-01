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
import frc.robot.Commands.CoralOutIntakeCommand;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
    /* Controllers */
    private final Joystick driver = new Joystick(0);
    //private final CoralOutIntakeSybsystem m_coralOutIntakeSybsystem = new CoralOutIntakeSybsystem();
    //private final CoralOutIntakeCommand m_CoralOutIntakeCommand = new CoralOutIntakeCommand(m_coralOutIntakeSybsystem);

   /* Driver Controls */
	private final int translationAxis = 1;
	private final int strafeAxis = 0;
    private final int triggerLeft = 2;
    private final int triggerRight = 3;
	private final int rotationAxis = 4;
    private final int leftShoulderButton = 5;
    private final int rightShoulderButton = 6;

    /* Driver Buttons */
    private final JoystickButton elevatorDisengage = new JoystickButton(driver, leftShoulderButton);
    private final JoystickButton elevatorEngage = new JoystickButton(driver, rightShoulderButton);

    private final JoystickButton zeroGyro = new JoystickButton(driver, XboxController.Button.kY.value);

    /* Subsystems */
    private final PoseEstimator s_PoseEstimator = new PoseEstimator();
    private final Swerve s_Swerve = new Swerve(s_PoseEstimator);
    private final ClimbSubsystem s_ClimbSubsystem = new ClimbSubsystem();
    private final CoralOutIntakeSubsystem s_CoralOutIntakeSubsystem = new CoralOutIntakeSubsystem();
    private final ElevatorSubsystem s_ElevatorSubsystem = new ElevatorSubsystem();
    private final Camera s_Camera = new Camera();
    //private final Vision s_Vision = new Vision(s_PoseEstimator);

    /* AutoChooser */
    //private final SendableChooser<Command> autoChooser;

    /** The container for the robot. Contains subsystems, OI devices, and commands. */
    public RobotContainer() {
        s_Swerve.setDefaultCommand(
            new SwerveCommand(
                s_Swerve, 
                () -> -driver.getRawAxis(translationAxis), 
                () -> driver.getRawAxis(strafeAxis),
                () -> -driver.getRawAxis(rotationAxis), 
                () -> false,
                () -> false,
                () -> 0 // Dynamic heading placeholder
            )
        );

        s_ClimbSubsystem.setDefaultCommand(
            new ClimbCommand(
              s_ClimbSubsystem, 
              () -> elevatorDisengage.getAsBoolean(), 
              () -> elevatorEngage.getAsBoolean()
            )
        );

        s_CoralOutIntakeSubsystem.setDefaultCommand(
            new CoralOutIntakeCommand(
              s_CoralOutIntakeSubsystem
              // ()->elevatorDisengage.getAsBoolean(), 
              // ()->elevatorEngage.getAsBoolean()
            )
        );

        s_ElevatorSubsystem.setDefaultCommand(
            new ElevatorCommand(
                s_ElevatorSubsystem,
                () -> driver.getRawAxis(triggerLeft),
                () -> driver.getRawAxis(triggerRight)
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
    } 

    /**
     * Use this to pass the autonomous command to the main {@link Robot} class.
     *
     * @return the command to run in autonomous
     */
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

// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;

import frc.robot.Commands.*;
import frc.robot.Controllers.ControllerInterface;
import frc.robot.Controllers.Logitech;
import frc.robot.Controllers.Xbox;
import frc.robot.Subsystems.*;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
    /* Controllers */
    private final ControllerInterface operator = new Logitech(0);
      //climb, elevator, and coral (control is blutooth)
    // private final ControllerInterface driver = new XboxController(1);
    private final ControllerInterface driver = new Logitech(1);
      // Controller in port 1 is driving (controll is cable)
    //private final CoralOutIntakeSybsystem m_coralOutIntakeSybsystem = new CoralOutIntakeSybsystem();
    //private final CoralOutIntakeCommand m_CoralOutIntakeCommand = new CoralOutIntakeCommand(m_coralOutIntakeSybsystem);

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
                () -> driver.getLeftYAxis(),
                () -> driver.getLeftXAxis(),
                () -> -driver.getRightXAxis(), 
                () -> false,
                () -> driver.getLeftBumper(),
                () -> 0.0, // Dynamic heading placeholder,
                () -> driver.getButtonY()
            )
        );
/*
        s_ClimbSubsystem.setDefaultCommand(
            new ClimbCommand(
              s_ClimbSubsystem, 
              ()->climbDisengage.getAsBoolean(),
              ()->climbEngage.getAsBoolean()
            )
        );
*/
         s_CoralOutIntakeSubsystem.setDefaultCommand(
             new CoralOutIntakeCommand(
               s_CoralOutIntakeSubsystem,
               () -> operator.getButtonX(),
               () -> operator.getButtonY()
               //() -> operator.getButtonA(),
               //() -> operator.getButtonB()
             )
         );

        s_ElevatorSubsystem.setDefaultCommand(
             new ElevatorCommand(
                 s_ElevatorSubsystem,
                () -> operator.getLeftTrigger(),
                () -> operator.getRightTrigger(),
                () -> operator.getLeftBumper(),
                () -> operator.getButtonA(),
                () -> operator.getButtonDPadDown(),
                () -> operator.getButtonDPadLeft(),
                () -> operator.getButtonDPadUp(),
                () -> operator.getButtonDPadRight()

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
     * edu.wpi.first.wpilibj.Joystick} or {@link Xbox}), and then passing it to a {@link
     * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
     */
    private void configureButtonBindings() {
        /* Driver Buttons */
        // zeroGyro.onTrue(new InstantCommand(() -> s_Swerve.zeroHeading()));

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

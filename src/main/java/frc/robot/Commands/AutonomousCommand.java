package frc.robot.Commands;

import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Subsystems.Swerve;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;


/*
SparkMax m_pivotMotor = new SparkMax(PIVOTion_MOTOR_ID, MotorType.kBrushless);
  SparkMax m_stealOrNoStealMotor = new SparkMax(ROTATION_MOTOR_ID, MotorType.kBrushless);
  SparkMaxConfig config = new SparkMaxConfig();

  AbsoluteEncoder m_pivotEncoder;
  AbsoluteEncoder m_rotationEncoder;

  DigitalInput m_limitSwitch = new DigitalInput(LIMITSWITCH_CHANNEL_ID);

  double m_pivotSetpoint;
  double PIVOT_MOTOR_SPEED = 0.5;

  PIDController m_PID = new PIDController(Kp, Ki, Kd);
  int STALL_CURRENT_THRESHOLD = 15; // amps

  /** Creates a new CoralIntakeOuttake. 
  public CoralOutIntakeSubsystem() {
    config.inverted(true);
    m_pivotMotor.configure(config, null, null);
    m_stealOrNoStealMotor.configure(config, null, null);
  }
 */

public class AutonomousCommand  extends SequentialCommandGroup {
    SparkMax m_testMotor4 = new SparkMax(4, MotorType.kBrushless);
    SparkMaxConfig config = new SparkMaxConfig();
    
    private final Swerve m_swerve;
    SmartDashboard m_smartdashboard;

    public AutonomousCommand(Swerve s_Swerve){
        addRequirements(s_Swerve);
        m_swerve = s_Swerve;

        config.inverted(true);
        m_testMotor4.configure(config, null, null);
    

        Timer autoTimer = new Timer();

        addCommands(
            //new InstantCommand(() -> m_swerve.drive(s_Swerve.getPose().getTranslation().plus(new Translation2d(1, 0)),
            // 0, isFinished(), isScheduled()),
            //new InstantCommand(() -> m_testMotor4.setVoltage(3))
            );
    }
}

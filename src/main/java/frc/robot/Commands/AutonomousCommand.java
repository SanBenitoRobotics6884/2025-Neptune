package frc.robot.Commands;

import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.HardwareConfigs;
import frc.robot.Subsystems.CoralOutIntakeSubsystem;
import frc.robot.Subsystems.ElevatorSubsystem;
import frc.robot.Subsystems.Swerve;

import com.revrobotics.spark.SparkLowLevel.MotorType;

import static frc.robot.Constants.Elevator.L1_POSITION;
import static frc.robot.Constants.Elevator.L3_POSITION;
import static frc.robot.Constants.Elevator.L4_POSITION;
import static frc.robot.Constants.Elevator.L5_POSITION;

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

public class AutonomousCommand  extends Command {
    private final Swerve m_swerve;
    private final CoralOutIntakeSubsystem m_CoralOutIntakeSubsystem;
    private final ElevatorSubsystem m_ElevatorSubsystem;
    Timer timer = new Timer();
    double autoTimer = timer.get();

    public AutonomousCommand(Swerve s_Swerve, CoralOutIntakeSubsystem s_CoralOutIntakeSubsystem, ElevatorSubsystem s_ElevatorSubsystem){
        addRequirements(s_Swerve);
        addRequirements(s_CoralOutIntakeSubsystem);
        addRequirements(s_ElevatorSubsystem);
        m_swerve = s_Swerve;
        m_CoralOutIntakeSubsystem = s_CoralOutIntakeSubsystem;
        m_ElevatorSubsystem = s_ElevatorSubsystem;

    }


    
    public void init (){
        timer.start();
    }
    public void execute(){
        
        timer.start();
       m_swerve.driveForward(-1);

        SmartDashboard.putNumber("a", timer.get());
        if(timer.get() > 6){
           
       m_ElevatorSubsystem.gotolevel(L4_POSITION-0.5);
        }
        if (timer.get() > 10){
           m_CoralOutIntakeSubsystem.intakeOuttake(false, false, true);
        }
        if (timer.get() > 10.1){
         m_ElevatorSubsystem.gotolevel(L5_POSITION);
        }
        if(timer.get() > 13){
        m_ElevatorSubsystem.ElevatorDownAuto();
        m_ElevatorSubsystem.gotolevel(L1_POSITION);
        }
        if(timer.get() > 17) {
            m_ElevatorSubsystem.ElevatorResetAuto();
        }
    }

    public boolean isFinished(){
        return (timer.get() - autoTimer) > 15;
      
    }

    public void end(boolean interrupted){
        m_swerve.driveForward(0);
    }
}

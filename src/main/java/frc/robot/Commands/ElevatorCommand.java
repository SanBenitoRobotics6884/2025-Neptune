package frc.robot.Commands;

import frc.robot.Constants;
import frc.robot.Subsystems.ElevatorSubsystem;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;


public class ElevatorCommand extends Command {
    private ElevatorSubsystem m_subsystem;
    private DoubleSupplier m_extendSup;
    private DoubleSupplier m_retractSup;

    public ElevatorCommand(ElevatorSubsystem s_subsystem, DoubleSupplier extendSup, DoubleSupplier retractSup) {
        m_subsystem = s_subsystem;
        addRequirements(m_subsystem);

        m_extendSup = extendSup;
        m_retractSup = retractSup;
    }

    @Override
    public void execute() {
        double extendVal = MathUtil.applyDeadband(extendSup.getAsDouble(), Constants.stickDeadband);
        double retractVal = MathUtil.applyDeadband(retractSup.getAsDouble(), Constants.stickDeadband);

        SmartDashboard.putNumber("Joystick extend", extendSup.getAsDouble());
        SmartDashboard.putNumber("Joystick retract", retractSup.getAsDouble());

        // current

        if(extendVal > 0){
          m_climbsubsystem.extend(extendVal);
        } else if (retractVal > 0){
          m_climbsubsystem.retract(retractVal);
        } else {
          m_subsystem.stop();
        }
    }

}
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
    private BooleanSupplier m_dampenSup;

    public ElevatorCommand(ElevatorSubsystem s_subsystem, DoubleSupplier extendSup, DoubleSupplier retractSup, BooleanSupplier dampenSup) {
        m_subsystem = s_subsystem;
        addRequirements(m_subsystem);

        m_extendSup = extendSup;
        m_retractSup = retractSup;
        m_dampenSup = dampenSup;
    }

    @Override
    public void execute() {
        double extendVal = MathUtil.applyDeadband(m_extendSup.getAsDouble()/4, Constants.stickDeadband) * (m_dampenSup.getAsBoolean() ? 0.2 : 1);
        double retractVal = MathUtil.applyDeadband(m_retractSup.getAsDouble()/4, Constants.stickDeadband) * (m_dampenSup.getAsBoolean() ? 0.2 : 1);

        SmartDashboard.putNumber("El extend", m_extendSup.getAsDouble());
        SmartDashboard.putNumber("El retract", m_retractSup.getAsDouble());

        // current

        if(extendVal > 0){
          m_subsystem.extend(extendVal);
        } else if (retractVal > 0){
          m_subsystem.retract(retractVal);
        } else {
          m_subsystem.stop();
        }
    }

}
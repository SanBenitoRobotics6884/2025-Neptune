package frc.robot.Commands;

import frc.robot.Constants;
import frc.robot.Subsystems.ElevatorSubsystem;

import static frc.robot.Constants.Elevator.L1_POSITION;
import static frc.robot.Constants.Elevator.L2_POSITION;
import static frc.robot.Constants.Elevator.L3_POSITION;
import static frc.robot.Constants.Elevator.L4_POSITION;

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
    private BooleanSupplier m_debugSup;
    private BooleanSupplier m_L1buttonSup;
    private BooleanSupplier m_L2buttonSup;
    private BooleanSupplier m_L3buttonSup;
    private BooleanSupplier m_L4buttonSup;
    private double LEVEL1_HEIGHT = 0.0;

    private Boolean m_debugMode;

    public ElevatorCommand(ElevatorSubsystem s_subsystem, DoubleSupplier extendSup, DoubleSupplier retractSup,
     BooleanSupplier dampenSup, BooleanSupplier debugSup, BooleanSupplier L1buttonSup, BooleanSupplier L2buttonSup,
     BooleanSupplier L3buttonSup, BooleanSupplier L4buttonSup) {
        m_subsystem = s_subsystem;
        addRequirements(m_subsystem);

        m_extendSup = extendSup;
        m_retractSup = retractSup;
        m_dampenSup = dampenSup;
        m_debugSup = debugSup;
        m_L1buttonSup = L1buttonSup;
        m_L2buttonSup = L2buttonSup;
        m_L3buttonSup = L3buttonSup;
        m_L4buttonSup = L4buttonSup;
    }

    @Override
    public void execute() {
      SmartDashboard.putBoolean("E EXECUTE", true);

        double extendVal = MathUtil.applyDeadband(m_extendSup.getAsDouble()/4, Constants.stickDeadband) * (m_dampenSup.getAsBoolean() ? 0.2 : 1);
        double retractVal = MathUtil.applyDeadband(m_retractSup.getAsDouble()/4, Constants.stickDeadband) * (m_dampenSup.getAsBoolean() ? 0.2 : 1);
        SmartDashboard.putNumber("El extend", m_extendSup.getAsDouble());
        SmartDashboard.putNumber("El retract", m_retractSup.getAsDouble());

        // current

        //enables debug mode for elevator while button held (turns off normal inputs); bound to A button
        m_debugMode = m_debugSup.getAsBoolean();

        if (!m_debugMode) {
          if (m_L1buttonSup.getAsBoolean()){
          m_subsystem.gotolevel(L1_POSITION);
              }
          if (m_L2buttonSup.getAsBoolean()){
            m_subsystem.gotolevel(L2_POSITION);
              }
          if (m_L3buttonSup.getAsBoolean()){
              m_subsystem.gotolevel(L3_POSITION);
              }
          if (m_L4buttonSup.getAsBoolean()){
                m_subsystem.gotolevel(L4_POSITION);
              }
          if(extendVal > 0){
            m_subsystem.extend(extendVal);
          } else if (retractVal > 0){
            m_subsystem.retract(retractVal);
          } else {
            m_subsystem.stop();
          }
        } else {
          m_subsystem.cycle();
        }
    }

}
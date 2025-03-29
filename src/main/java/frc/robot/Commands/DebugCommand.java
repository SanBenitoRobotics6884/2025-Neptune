package frc.robot.Commands;

import frc.robot.Constants;
import frc.robot.Subsystems.Swerve;
import frc.robot.Subsystems.DebugSubsystem;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;



import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;

import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.PositionDutyCycle;
import com.ctre.phoenix6.configs.TalonFXSConfiguration;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.hardware.TalonFXS;
import com.ctre.phoenix6.signals.MotorArrangementValue;
import frc.lib.math.Conversions;
import frc.lib.util.swerveUtil.SwerveModuleConstants;
import com.ctre.phoenix6.controls.PositionVoltage;


public class DebugCommand extends Command {
    private DebugSubsystem m_subsystem;
    private BooleanSupplier m_resetSup;
    private BooleanSupplier m_abutton;
    private BooleanSupplier m_bbutton;

    public DebugCommand(DebugSubsystem subsystem, BooleanSupplier resetSup, BooleanSupplier a, BooleanSupplier b) {
        m_subsystem = subsystem;
        addRequirements(m_subsystem);
        m_resetSup = resetSup;
        m_abutton = a; // Set angle to 2 rotations when A is pressed
        m_bbutton = b; // Set angle to -2 rotations when B is pressed
    }
    @Override
    public void execute() {
        if(m_resetSup.getAsBoolean()) {
            m_subsystem.zero();
       }
       if(m_abutton.getAsBoolean()){
            m_subsystem.setAngle(2);
       }
       if (m_bbutton.getAsBoolean()){
        m_subsystem.setAngle(-2);
       }
    }
}




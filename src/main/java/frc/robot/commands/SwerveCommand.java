package frc.robot.Commands;

import frc.robot.Constants;
import frc.robot.Subsystems.Swerve;
import frc.robot.Subsystems.CoralOutIntakeSubsystem;
import frc.robot.Subsystems.SwerveMod;

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


public class SwerveCommand extends Command {    
    private Swerve s_Swerve;    
    private CoralOutIntakeSubsystem s_CoralOutIntakeSubsystem;
    private DoubleSupplier translationSup;
    private DoubleSupplier strafeSup;
    private DoubleSupplier rotationSup;
    private DoubleSupplier dynamicHeadingSup;
    private BooleanSupplier robotCentricSup;
    private BooleanSupplier debugDirectSup;
    private BooleanSupplier debugSup;
    private BooleanSupplier dampenSup;
    private BooleanSupplier zeroGyro;
    private PIDController rotationController;
    

    public SwerveCommand(Swerve s_Swerve, CoralOutIntakeSubsystem s_CoralOutIntakeSubsystem, DoubleSupplier translationSup, DoubleSupplier strafeSup, DoubleSupplier rotationSup, BooleanSupplier robotCentricSup, BooleanSupplier dampen, DoubleSupplier dynamicHeadingSup, BooleanSupplier zeroGyro, BooleanSupplier debugSup, BooleanSupplier debugDirectSup) {
        this.s_Swerve = s_Swerve;
        addRequirements(s_Swerve);
        this.s_CoralOutIntakeSubsystem = s_CoralOutIntakeSubsystem;
        addRequirements(s_CoralOutIntakeSubsystem);

        //TODO: Tune heading PID
        rotationController = new PIDController(Constants.Swerve.HeadingKP, Constants.Swerve.HeadingKI, Constants.Swerve.HeadingKD );
        rotationController.enableContinuousInput(-Math.PI, Math.PI);
        rotationController.setTolerance(Constants.Swerve.HeadingTolerence);

        this.translationSup = translationSup;
        this.strafeSup = strafeSup;
        this.rotationSup = rotationSup;
        this.robotCentricSup = robotCentricSup;
        this.dampenSup = dampen;
        this.dynamicHeadingSup = dynamicHeadingSup;
        this.zeroGyro = zeroGyro;
        this.debugSup = debugSup;
        this.debugDirectSup = debugDirectSup;
    }

    @Override
    public void execute() {
        if(zeroGyro.getAsBoolean()){
            s_Swerve.zeroHeading();
        }
        /* Drive */
        if (!DriverStation.isAutonomous()){
            if (debugSup.getAsBoolean()){
                debugModules();
            }
            else if(debugDirectSup.getAsBoolean()){
                debugModulesDirect();
            } else {
                drive();
            }
        }
    }

    public void drive() {
        double translationVal = MathUtil.applyDeadband(translationSup.getAsDouble(), Constants.stickDeadband) * (dampenSup.getAsBoolean() ? 0.2 : 1);
        double strafeVal = -MathUtil.applyDeadband(strafeSup.getAsDouble(), Constants.stickDeadband) * (dampenSup.getAsBoolean() ? 0.2 : 1);
        double rotationVal = MathUtil.applyDeadband(rotationSup.getAsDouble(), Constants.stickDeadband) * (dampenSup.getAsBoolean() ? 0.2 : 1);

        SmartDashboard.putNumber("S translation", translationVal);
        SmartDashboard.putNumber("S strafe", strafeVal);
        SmartDashboard.putNumber("S rotation", rotationVal);
        SmartDashboard.putBoolean("S dampenSup", dampenSup.getAsBoolean());
        rotationVal = rotationVal * Constants.Swerve.maxAngularVelocity;
        if(s_CoralOutIntakeSubsystem.getSpeed() > 0.1){
            s_Swerve.drive(
                new Translation2d(translationVal, strafeVal).times(Constants.Swerve./**Swerve*/maxSpeed),
                rotationVal,
                !robotCentricSup.getAsBoolean(), 
                true
            );        
        }
    }

    public void debugModules() {
        double speedMps = (System.currentTimeMillis() / 1000 ) % 5;
        double angle = ((System.currentTimeMillis() / 1000 ) % 8);
        angle *= 45;

        for (int i=0; i <= 3; i++){
            SwerveMod mod = s_Swerve.mSwerveMods[i];
            SwerveModuleState state = new SwerveModuleState(0.5, Rotation2d.fromDegrees(180));
            mod.setDesiredState(state, true);
            System.out.println("debugSwerveDirect (" + i + ") - " + speedMps + " - " + angle);
        }
    }

    public void debugModulesDirect() {
        for (int i=0; i <= 3; i++){
            SwerveMod mod = s_Swerve.mSwerveMods[i];

            double speedMps = (System.currentTimeMillis() / 1000 ) % 5;
            DutyCycleOut driveDutyCycle = new DutyCycleOut(0);
            driveDutyCycle.Output = speedMps / Constants.Swerve.maxSpeed;
            mod.mDriveMotor.setControl(driveDutyCycle);


            double angle = (System.currentTimeMillis() / 1000 ) % 8;
            angle *= 45;

            double gearRatio = 150.0 / 7;
            double targetAngle = Rotation2d.fromDegrees(angle).getRotations() * gearRatio;
            PositionDutyCycle position = new PositionDutyCycle(targetAngle);
            mod.mAngleMotor.setControl(position.withSlot(0));

            System.out.println("debugSwerveDirect (" + i + ") - " + speedMps + " - " + angle);
        }
    }
}
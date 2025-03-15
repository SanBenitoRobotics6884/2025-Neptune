package frc.robot.Commands;

import frc.robot.Constants;
import frc.robot.Subsystems.Swerve;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;


public class SwerveCommand extends Command {    
    private Swerve s_Swerve;    
    private DoubleSupplier translationSup;
    private DoubleSupplier strafeSup;
    private DoubleSupplier rotationSup;
    private DoubleSupplier dynamicHeadingSup;
    private BooleanSupplier robotCentricSup;
    private BooleanSupplier dampenSup;
    private BooleanSupplier zeroGyro;
    private PIDController rotationController;
    

    public SwerveCommand(Swerve s_Swerve, DoubleSupplier translationSup, DoubleSupplier strafeSup, DoubleSupplier rotationSup, BooleanSupplier robotCentricSup, BooleanSupplier dampen, DoubleSupplier dynamicHeadingSup, BooleanSupplier zeroGyro) {
        this.s_Swerve = s_Swerve;
        addRequirements(s_Swerve);

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
    }

    @Override
    public void execute() {
        double translationVal = MathUtil.applyDeadband(translationSup.getAsDouble(), Constants.stickDeadband) * (dampenSup.getAsBoolean() ? 0.2 : 1);
        double strafeVal = -MathUtil.applyDeadband(strafeSup.getAsDouble(), Constants.stickDeadband) * (dampenSup.getAsBoolean() ? 0.2 : 1);
        double rotationVal = MathUtil.applyDeadband(rotationSup.getAsDouble(), Constants.stickDeadband) * (dampenSup.getAsBoolean() ? 0.2 : 1);

        SmartDashboard.putNumber("S translation", translationSup.getAsDouble());
        SmartDashboard.putNumber("S strafe", strafeSup.getAsDouble());
        SmartDashboard.putNumber("S rotation", rotationSup.getAsDouble());
        SmartDashboard.putBoolean("S dampenSup", dampenSup.getAsBoolean());
        rotationVal = rotationVal * Constants.Swerve.maxAngularVelocity;

        if(zeroGyro.getAsBoolean()){
            s_Swerve.zeroHeading();
        }
        /* Drive */
        if (!DriverStation.isAutonomous()){
            s_Swerve.drive(
                new Translation2d(translationVal, strafeVal).times(Constants.Swerve./**Swerve*/maxSpeed),
                rotationVal,
                !robotCentricSup.getAsBoolean(), 
                true
            );    
        }
    }
}
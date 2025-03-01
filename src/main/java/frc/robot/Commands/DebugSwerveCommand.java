package frc.robot.Commands;

import frc.robot.Constants;
import frc.robot.Subsystems.Swerve;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import com.ctre.phoenix6.controls.DutyCycleOut;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkClosedLoopController;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;


public class DebugSwerveCommand extends Command {
    private Swerve s_Swerve;

    public DebugSwerveCommand(Swerve s_Swerve) {
        this.s_Swerve = s_Swerve;
        addRequirements(s_Swerve);
    }

    @Override
    public void execute() {
        /* Get Values, Deadband, Dampen */

        for (int i=0; i < 3; i++){
            double speedMps = (System.currentTimeMillis() / 1000 ) % 5;
            DutyCycleOut driveDutyCycle = new DutyCycleOut(0);
            driveDutyCycle.Output = speedMps / Constants.Swerve.maxSpeed;
            s_Swerve.mSwerveMods[i].mDriveMotor.setControl(driveDutyCycle);
            System.out.println("setting speed to " + speedMps);

            SparkClosedLoopController angleController =  s_Swerve.mSwerveMods[i].mAngleMotor.getClosedLoopController();
            angleController.setReference(
                speedMps * 10,         // setpoint
                ControlType.kPosition,  // position closed-loop
                ClosedLoopSlot.kSlot0   // uses PID slot 0
            );
            System.out.println("setting angle to " + speedMps * 10);
        }
   }
}
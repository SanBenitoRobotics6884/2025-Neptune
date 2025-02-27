package frc.robot;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import frc.lib.util.swerveUtil.SwerveModuleConstants;

import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;

import frc.lib.math.Conversions;
/**
 * a Swerve Modules using REV Robotics motor controllers and CTRE CANcoder absolute encoders.
 */
public class SwerveMod{
    private HardwareConfigs hardwareConfigs;

    /** Which module # is this? (0,1,2,3...) */
    public int moduleNumber;

    /** Offset from your absolute encoder to "zero" the angle */
    private Rotation2d angleOffset;

    /** SparkMax for the steering (angle) NEO */
    private SparkMax mAngleMotor;
    private RelativeEncoder relAngleEncoder;

    /** TalonFX for the drive */
    private TalonFX mDriveMotor;

    /** CTRE CANcoder for absolute angle measurement */
    private CANcoder angleEncoder;
    private final DutyCycleOut driveDutyCycle = new DutyCycleOut(0);
    private final VelocityVoltage driveVelocity = new VelocityVoltage(0);
    private final SimpleMotorFeedforward driveFeedForward = new SimpleMotorFeedforward(Constants.Swerve.driveKS, Constants.Swerve.driveKV, Constants.Swerve.driveKA);


    public SwerveMod(int moduleNumber, SwerveModuleConstants moduleConstants) {
        this.hardwareConfigs = Robot.hardwareConfigs;
        this.moduleNumber    = moduleNumber;
        this.angleOffset     = moduleConstants.angleOffset;

        // -----------------------------------------------------
        // Absolute angle encoder (CTRE CANcoder)
        // -----------------------------------------------------
        angleEncoder = new CANcoder(moduleConstants.cancoderID, "Galigma");
        // Apply your CANcoder config (sensorCoefficient, magnetOffset, etc.)
        angleEncoder.getConfigurator().apply(hardwareConfigs.swerveCANcoderConfig);

        // -----------------------------------------------------
        // Angle (steering) Motor – NEO w/ SparkMax
        // -----------------------------------------------------
        mAngleMotor = new SparkMax(moduleConstants.angleMotorID, MotorType.kBrushless);
        // Load your SparkMax angle config (PID, current limit, ramp, etc.)
        mAngleMotor.configure(
            hardwareConfigs.swerveAngleSparkConfig,
            ResetMode.kNoResetSafeParameters,
            PersistMode.kPersistParameters
        );
        relAngleEncoder = mAngleMotor.getEncoder();

        // -----------------------------------------------------
        // Drive Motor – TalonFX
        // -----------------------------------------------------
        mDriveMotor = new TalonFX(moduleConstants.driveMotorID, "Galigma");
        // Load your TalonFX drive config (PID, current limit, etc.)
        mDriveMotor.getConfigurator().apply(hardwareConfigs.swerveDriveTalonConfig);
        mDriveMotor.getConfigurator().setPosition(0.0);

        // Initialize encoders
        configEncoders();
    }

    private void configEncoders() {
        // For drive motor (TalonFX), start integrated sensor at 0
        mDriveMotor.setPosition(0.0);

        // For angle motor (SparkMax), we will zero to the absolute CANcoder reading
        resetToAbsolute();
    }

    /**
     * Called every cycle to command the swerve module.
     *
     * @param desiredState The target angle (in SwerveModuleState.angle) and speed (in speedMetersPerSecond)
     * @param isOpenLoop   If true, run drive motor in simple duty cycle mode; otherwise run velocity closed-loop.
     */
    public void setDesiredState(SwerveModuleState desiredState, boolean isOpenLoop) {
        // 1) Optimize the commanded angle to avoid unnecessary rotation
        // desiredState = CTREModuleState.optimize(desiredState, getState().angle);
        // desiredState = SwerveModuleState.optimize(desiredState, getState().angle);

        // 2) Set the steering angle (SparkMax)
        setAngle(desiredState);

        // 3) Set the drive speed (TalonFX)
        setSpeed(desiredState, isOpenLoop);

        SmartDashboard.putNumber("Desired angle (deg) - Mod " + moduleNumber,
                                 desiredState.angle.getDegrees());
    }

    private void setSpeed(SwerveModuleState desiredState, boolean isOpenLoop){
        SmartDashboard.putNumber("Desired Speed " + moduleNumber, desiredState.speedMetersPerSecond);
        double speedMps = desiredState.speedMetersPerSecond;
        if(isOpenLoop){
            driveDutyCycle.Output = speedMps / Constants.Swerve.maxSpeed;
            mDriveMotor.setControl(driveDutyCycle);
            SmartDashboard.putNumber("Desired Duty Cycle " + moduleNumber, driveDutyCycle.Output);
            System.out.println("Desired Duty Cycle: " + driveDutyCycle.Output);
        }
        else {
            driveVelocity.Velocity = Conversions.MPSToRPS(desiredState.speedMetersPerSecond, Constants.Swerve.wheelCircumference);
            driveVelocity.FeedForward = driveFeedForward.calculate(desiredState.speedMetersPerSecond);
            SmartDashboard.putNumber("Desired Velocity " + moduleNumber, driveVelocity.Velocity);
            System.out.println("Desired Velocity: " + driveVelocity.Velocity);
            mDriveMotor.setControl(driveVelocity.withSlot(0));
        }
        SmartDashboard.putNumber("Actual Speed " + moduleNumber, mDriveMotor.getVelocity().getValueAsDouble());
    }

    // -----------------------------------------------------
    // Angle control (SparkMax)
    // -----------------------------------------------------
    private void setAngle(SwerveModuleState desiredState) {
        // If the module is barely driving, don’t waste time spinning the angle
        if (Math.abs(desiredState.speedMetersPerSecond) <= (Constants.Swerve.maxSpeed * 0.01)) {
            // Optionally hold position or just stop sending commands:
            mAngleMotor.stopMotor();
            return;
        }

        // We want to steer the module to the desired angle in degrees
        double targetAngleDeg = desiredState.angle.getDegrees();

        // By default, SparkMax’s RelativeEncoder is in rotations
        // If you want to use degrees, you could set a conversion factor:
        //   relAngleEncoder.setPositionConversionFactor(360.0);
        // Then your angle = relAngleEncoder.getPosition() would be degrees
        // If so, then passing targetAngleDeg to setReference() is correct.

        SparkClosedLoopController angleController = mAngleMotor.getClosedLoopController();
        angleController.setReference(
            targetAngleDeg,         // setpoint
            ControlType.kPosition,  // position closed-loop
            ClosedLoopSlot.kSlot0   // uses PID slot 0
        );
    }

    // -----------------------------------------------------
    // Sensor reading / getters
    // -----------------------------------------------------
    /** Current angle from the NEO (SparkMax integrated encoder), as a Rotation2d. */
    private Rotation2d getAngle() {
        // If you set setPositionConversionFactor(360.0) for the angle SparkMax,
        // then relAngleEncoder.getPosition() is in degrees.
        double angleDeg = relAngleEncoder.getPosition();
        return Rotation2d.fromDegrees(angleDeg);
    }

    /** Absolute angle from the CANcoder, always in the [0,360) domain if you convert rotations → degrees. */
    public Rotation2d getCANcoder() {
        double absPositionRot = angleEncoder.getAbsolutePosition().getValueAsDouble(); // 0..1 rotations
        SmartDashboard.putNumber("Absolute Angle" + moduleNumber, absPositionRot);
        double absPositionDeg = absPositionRot * 360.0;
        return Rotation2d.fromDegrees(absPositionDeg);
    }

    /**
     * Resets the SparkMax’s integrated steering encoder to the absolute angle, minus the known offset.
     * This is typically called once at robot init.
     */
    public void resetToAbsolute() {
        double absAngleDeg   = getCANcoder().getDegrees();
        double adjustedAngle = absAngleDeg - angleOffset.getDegrees();

        // If your SparkMax is set to a 1:1 factor in rotations (the default),
        // then calling setPosition(X) is in rotations. So if we want to store
        // everything in degrees, we must setPosition(X / 360).
        // But if you already setPositionConversionFactor(360.0), then
        // setPosition(adjustedAngle) is correct.
        //
        // For example:
        // relAngleEncoder.setPositionConversionFactor(360.0); // 1 rotation = 360 degrees
        // => setPosition in degrees:
        SmartDashboard.putNumber("Reset Angle " + moduleNumber, adjustedAngle);
        relAngleEncoder.setPosition(adjustedAngle);

        // If you do NOT set that conversion factor, you’d do:
        // relAngleEncoder.setPosition(adjustedAngle / 360.0);
    }

    /**
     * Returns the module’s overall state:
     *  - Speed (m/s) read from the TalonFX
     *  - Angle read from the SparkMax
     */
    public SwerveModuleState getState(){
        return new SwerveModuleState(
            Conversions.RPSToMPS(mDriveMotor.getVelocity().getValueAsDouble(), Constants.Swerve.wheelCircumference),
            getAngle()
        );
    }
    /**
     * Returns the module’s position:
     *  - Drive distance (meters) from the TalonFX integrated sensor
     *  - Angle from SparkMax
     */
    public SwerveModulePosition getPosition(){
        return new SwerveModulePosition(
            Conversions.rotationsToMeters(mDriveMotor.getPosition().getValueAsDouble(), Constants.Swerve.wheelCircumference),
            getAngle()
        );
    }

    // -----------------------------------------------------
    // Boilerplate
    // -----------------------------------------------------
    public int getModuleNumber() {
        return moduleNumber;
    }

    public void setModuleNumber(int moduleNumber) {
        this.moduleNumber = moduleNumber;
    }
}

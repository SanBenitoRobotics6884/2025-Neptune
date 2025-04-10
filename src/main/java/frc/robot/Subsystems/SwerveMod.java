package frc.robot.Subsystems;

import frc.robot.Constants;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;

import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.PositionDutyCycle;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.MotorArrangementValue;
import frc.lib.math.Conversions;
import frc.lib.util.swerveUtil.SwerveModuleConstants;
import com.ctre.phoenix6.controls.PositionVoltage;

/**
 * a Swerve Modules using REV Robotics motor controllers and CTRE CANcoder
 * absolute encoders.
 */

public class SwerveMod {
    private String CANIVOR_BUS = "Galigma";

    /** Which module # is this? (0,1,2,3...) */
    public int moduleNumber;

    /** Offset from your absolute encoder to "zero" the angle */
    private Rotation2d angleOffset;

    /** TalonFX Motor Controller for the steering (angle) NEO */
    public TalonFX mAngleMotor;

    /** TalonFX for the drive */
    public TalonFX mDriveMotor;

    /** CTRE CANcoder for absolute angle measurement */
    private CANcoder angleEncoder;
    private final DutyCycleOut driveDutyCycle = new DutyCycleOut(0);
    private final VelocityVoltage driveVelocity = new VelocityVoltage(0);
    private final SimpleMotorFeedforward driveFeedForward = new SimpleMotorFeedforward(Constants.Swerve.driveKS,
            Constants.Swerve.driveKV, Constants.Swerve.driveKA);

    public SwerveMod(int moduleNumber, SwerveModuleConstants moduleConstants) {
        this.moduleNumber = moduleNumber;
        this.angleOffset = moduleConstants.angleOffset;

        // -----------------------------------------------------
        // Absolute angle encoder (CTRE CANcoder)
        // -----------------------------------------------------
        // Apply your CANcoder config (sensorCoefficient, magnetOffset, etc.)
        angleEncoder = new CANcoder(moduleConstants.cancoderID, CANIVOR_BUS);
        // config.AbsoluteSensorRange = AbsoluteSensorRangeValue.Signed_PlusMinusHalf;
        angleEncoder.getAbsolutePosition().setUpdateFrequency(100);
        angleEncoder.optimizeBusUtilization();
        angleEncoder.getConfigurator().apply(moduleConstants.asMagnetSensorConfig());

        // -----------------------------------------------------
        // Angle (steering) Motor – Kraken
        // -----------------------------------------------------
        mAngleMotor = new TalonFX(moduleConstants.angleMotorID, CANIVOR_BUS);
        mAngleMotor.getConfigurator().apply(moduleConstants.asTalonSteerConfig());
        // mAngleMotor.getConfigurator().setPosition(0.0);

        // -----------------------------------------------------
        // Drive Motor – Kraken
        // -----------------------------------------------------
        mDriveMotor = new TalonFX(moduleConstants.driveMotorID, CANIVOR_BUS);
        // Load your TalonFX drive config (PID, current limit, etc.)
        mDriveMotor.getConfigurator().apply(moduleConstants.asTalonConfig());
        mDriveMotor.getConfigurator().setPosition(0.0);

        // Initialize encoders
        configEncoders();
    }

    private void configEncoders() {
        // For drive motor (TalonFX), start integrated sensor at 0
        mDriveMotor.setPosition(0.0);

        resetToAbsolute();
        SwerveModuleState idleState = new SwerveModuleState(0.0, Rotation2d.fromDegrees(180));
        setDesiredState(idleState, true);
    }

    /**
     * Called every cycle to command the swerve module.
     *
     * @param desiredState The target angle (in SwerveModuleState.angle) and speed
     *                     (in speedMetersPerSecond)
     * @param isOpenLoop   If true, run drive motor in simple duty cycle mode;
     *                     otherwise run velocity closed-loop.
     */
    public void setDesiredState(SwerveModuleState desiredState, boolean isOpenLoop) {
        // 1) Optimize the commanded angle to avoid unnecessary rotation
        // desiredState = CTREModuleState.optimize(desiredState, getState().angle);
        // desiredState = SwerveModuleState.optimize(desiredState, getState().angle);

        // 2) Set the steering angle (TalonFX Motor Controller)
        setAngle(desiredState);

        // 3) Set the drive speed (TalonFX)
        setSpeed(desiredState, isOpenLoop);
    }

    private void setSpeed(SwerveModuleState desiredState, boolean isOpenLoop) {
        double speedMps = desiredState.speedMetersPerSecond;
        if (isOpenLoop) {
            driveDutyCycle.Output = speedMps / Constants.Swerve.maxSpeed;
            mDriveMotor.setControl(driveDutyCycle);
        } else {
            driveVelocity.Velocity = Conversions.MPSToRPS(desiredState.speedMetersPerSecond,
                    Constants.Swerve.wheelCircumference);
            driveVelocity.FeedForward = driveFeedForward.calculate(desiredState.speedMetersPerSecond);
            mDriveMotor.setControl(driveVelocity.withSlot(0));
        }
    }

    // -----------------------------------------------------
    // Angle control
    // -----------------------------------------------------
    private void setAngle(SwerveModuleState desiredState) {
        // If the module is barely driving, don’t waste time spinning the angle
        // if (Math.abs(desiredState.speedMetersPerSecond) <= (Constants.Swerve.maxSpeed
        // * 0.01)) {
        // // Optionally hold position or just stop sending commands:
        // mAngleMotor.stopMotor();
        // return;
        // }

        // We want to steer the module to the desired angle in degrees
        //double targetAngle = desiredState.angle.getDegrees();
        double rots = desiredState.angle.getRotations();
        double curRots = mAngleMotor.getPosition().getValueAsDouble();

        double diff = rots - curRots;
        // if(Math.abs(diff) > 0.5){
        //     if(diff > 0){
        //         rots -= 1;
        //     } else {
        //         rots += 1;
        //     }
        // }

        double targetAngle = rots;

        PositionDutyCycle position = new PositionDutyCycle(targetAngle);
        mAngleMotor.setControl(position.withSlot(0));

    }

    // -----------------------------------------------------
    // Sensor reading / getters
    // -----------------------------------------------------
    /** Current angle from the angle relative encoder, as a Rotation2d. */
    private Rotation2d getAngle() {
        return Rotation2d.fromRotations(mAngleMotor.getPosition().getValueAsDouble());
    }

    /**
     * Absolute angle from the CANcoder, always in the [0,360) domain if you convert
     * rotations → degrees.
     */
    public Rotation2d getCANcoder() {
        double absPositionRot = 0;
        for(int i = 0; i < 10; i++){
           // angleEncoder.waitForUpdate(10);
            absPositionRot = angleEncoder.getAbsolutePosition().getValueAsDouble(); // 0..1 rotations
            if(Math.abs(absPositionRot) > 0.001){
                break;
            }
        }
        return Rotation2d.fromRotations(absPositionRot);
    }

    /**
     * Resets the TalonFX Motor Controller’s integrated steering encoder to the
     * absolute angle, minus the known offset.
     * This is typically called once at robot init.
     */
    public void resetToAbsolute() {
        // This is where you can find the real absolute angles for the CanCoder.
        // During Swerve tuning, you can use this to find the offset.
        double absAngle = getCANcoder().getRotations();
        double adjustedAngle = absAngle - angleOffset.getRotations();
        double newPosition = -adjustedAngle;
        StatusCode sc = mAngleMotor.setPosition(newPosition, 10);
        mAngleMotor.getPosition().waitForUpdate(10.0);
    }

    /**
     * Returns the module’s overall state:
     * - Speed (m/s) read from the TalonFX
     * - Angle read from the TalonFX Motor Controller
     */
    public SwerveModuleState getState() {
        return new SwerveModuleState(
                Conversions.RPSToMPS(mDriveMotor.getVelocity().getValueAsDouble(), Constants.Swerve.wheelCircumference),
                getAngle());
    }

    /**
     * Returns the module’s position:
     * - Drive distance (meters) from the TalonFX integrated sensor
     * - Angle from TalonFX Motor Controller
     */
    public SwerveModulePosition getPosition() {
        return new SwerveModulePosition(
                Conversions.rotationsToMeters(mDriveMotor.getPosition().getValueAsDouble(),
                        Constants.Swerve.wheelCircumference),
                getAngle());
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

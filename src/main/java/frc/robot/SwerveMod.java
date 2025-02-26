package frc.robot;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import frc.lib.util.swerveUtil.CTREModuleState;
import frc.lib.util.swerveUtil.SwerveModuleConstants;

import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;

public class SwerveMod {
    private HardwareConfigs hardwareConfigs;

    private int moduleNumber;
    private Rotation2d angleOffset;

    // TalonFX for angle & drive
    private TalonFX mAngleMotor;
    private TalonFX mDriveMotor;

    // CANcoder for absolute angle
    private CANcoder angleEncoder;

    public SwerveMod(int moduleNumber, SwerveModuleConstants moduleConstants) {
        this.hardwareConfigs = Robot.hardwareConfigs;
        this.moduleNumber    = moduleNumber;
        this.angleOffset     = moduleConstants.angleOffset;

        /* Absolute angle encoder (CANcoder) */
        angleEncoder = new CANcoder(moduleConstants.cancoderID, "rio");
        angleEncoder.getConfigurator().apply(hardwareConfigs.swerveCANcoderConfig);

        /* Angle Motor (TalonFX) */
        mAngleMotor = new TalonFX(moduleConstants.angleMotorID, "rio");
        mAngleMotor.getConfigurator().apply(hardwareConfigs.swerveAngleTalonConfig);

        /* Drive Motor (TalonFX) */
        mDriveMotor = new TalonFX(moduleConstants.driveMotorID, "rio");
        mDriveMotor.getConfigurator().apply(hardwareConfigs.swerveDriveTalonConfig);

        // Initialize sensors
        configEncoders();
    }

    private void configEncoders() {
        // Make sure the integrated sensors start at 0 or the desired absolute position
        mDriveMotor.setRotorPosition(0.0);
        resetToAbsolute(); // set angle motor’s integrated sensor to match CANcoder
    }

    /**
     * Main control entry point:
     * 1) Optimize SwerveModuleState (to avoid rotating more than 90 deg, etc.)
     * 2) Set angle
     * 3) Set drive speed (open-loop or closed-loop).
     */
    public void setDesiredState(SwerveModuleState desiredState, boolean isOpenLoop) {
        // Optimize reference state
        desiredState = CTREModuleState.optimize(desiredState, getState().angle);

        setAngle(desiredState);
        setSpeed(desiredState, isOpenLoop);

        SmartDashboard.putNumber("Desired angle (deg) [" + moduleNumber + "]",
                                 desiredState.angle.getDegrees());
    }

    /**
     * Sets drive motor, either open-loop (percent output) or closed-loop (velocity).
     */
    private void setSpeed(SwerveModuleState desiredState, boolean isOpenLoop) {
        double speed = desiredState.speedMetersPerSecond;

        if (isOpenLoop) {
            // Open-loop: percent output vs. max speed
            double percentOutput = speed / Constants.Swerve.maxSpeed;
            mDriveMotor.setControl(new DutyCycleOut(percentOutput));
        } else {
            // Closed-loop: velocity control in your configured sensor units
            // Make sure your hardware config or code converts from m/s to “sensor units per 100ms” or “rotations/sec”
            mDriveMotor.setControl(
                new VelocityVoltage(speed)
                    .withSlot(0) // use whichever closed-loop slot you have tuned
            );
        }
    }

    /**
     * Sets angle using position closed-loop.
     * Exits early if the wheel speed is nearly zero to avoid “twitching”.
     */
    private void setAngle(SwerveModuleState desiredState) {
        if (Math.abs(desiredState.speedMetersPerSecond) <= (Constants.Swerve.maxSpeed * 0.01)) {
            // If drive speed is very small, don’t bother reorienting
            mAngleMotor.setControl(new DutyCycleOut(0.0));
            return;
        }

        // We want the final angle in degrees:
        double targetAngleDeg = desiredState.angle.getDegrees();

        // In Phoenix 6, if your sensor coefficient is set so that
        // 1 rotation of the motor = 360 sensor units, you can pass degrees directly.
        // Otherwise, you’ll need to convert from degrees → sensor units or rotations.
        mAngleMotor.setControl(
            new PositionVoltage(targetAngleDeg)
                .withSlot(0) // whichever slot has your angle PID gains
        );
    }

    /**
     * Returns the module's *integrated* angle (from the TalonFX sensor),
     * as a Rotation2d.
     */
    private Rotation2d getAngle() {
        // If your sensor is configured for “degrees” via sensorCoefficient=360,
        // then getPosition().getValue() is already in degrees. Otherwise, convert as needed.
        double angleDeg = mAngleMotor.getPosition().getValue();
        return Rotation2d.fromDegrees(angleDeg);
    }

    /**
     * Returns the absolute angle from the CANcoder.
     */
    public Rotation2d getCANcoder() {
        // By default, CANcoder returns [0.0, 1.0) rotations. Convert to degrees or keep as rotations:
        double absolutePosRot = angleEncoder.getAbsolutePosition().getValueAsDouble();
        // Convert rotations to degrees:
        return Rotation2d.fromDegrees(absolutePosRot * 360.0);
    }

    /**
     * Reset the TalonFX’s integrated angle sensor so that it matches the absolute CANcoder angle,
     * minus your known mechanical offset.
     */
    public void resetToAbsolute() {
        double absoluteAngleDeg = getCANcoder().getDegrees();
        double adjustedAngleDeg = absoluteAngleDeg - angleOffset.getDegrees();

        // If sensorCoefficient is 360 = 1 full rotation, setRotorPosition() can take degrees directly:
        // Otherwise, convert degrees → rotations or sensor ticks as needed.
        mAngleMotor.setRotorPosition(adjustedAngleDeg);
    }

    public SwerveModuleState getState() {
        // Velocity from the integrated drive sensor
        // Make sure you convert from raw sensor units → m/s
        double driveVel = mDriveMotor.getVelocity().getValue();
        Rotation2d angle = getAngle();

        return new SwerveModuleState(driveVel, angle);
    }

    public SwerveModulePosition getPosition() {
        // Position from drive motor integrated sensor
        // Again, ensure you convert from rotations (or ticks) → meters
        double drivePos = mDriveMotor.getPosition().getValue();
        Rotation2d angle = getAngle();

        return new SwerveModulePosition(drivePos, angle);
    }

    public int getModuleNumber() {
        return moduleNumber;
    }

    public void setModuleNumber(int moduleNumber) {
        this.moduleNumber = moduleNumber;
    }
}

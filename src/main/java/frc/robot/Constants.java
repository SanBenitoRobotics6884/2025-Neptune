package frc.robot;

import com.ctre.phoenix6.signals.SensorDirectionValue;
import com.pathplanner.lib.config.ModuleConfig;
import com.pathplanner.lib.config.PIDConstants;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.LEDPattern;
import edu.wpi.first.wpilibj.util.Color;
import frc.lib.util.swerveUtil.COTSTalonFXSwerveConstants;
import frc.lib.util.swerveUtil.COTSNeoSwerveConstants;
import frc.lib.util.swerveUtil.SwerveModuleConstants;
import frc.lib.util.swerveUtil.COTSNeoSwerveConstants.driveGearRatios;
import com.ctre.phoenix6.signals.NeutralModeValue;

import static edu.wpi.first.units.Units.Meters;

import com.ctre.phoenix6.signals.InvertedValue;

public final class Constants {
    public static final double stickDeadband = 0.1;
    public static final String GALIGMA_BUS = "Galigma"; // Thanks, Julio for all the troubles. :( -Zach

    public static final class Swerve {
        public static final int pigeonID = 60;
        public static final boolean invertGyro = false; // Always ensure Gyro is CCW+ CW-

        public static final COTSNeoSwerveConstants chosenModule =  //TODO: This must be tuned to specific robot
        COTSNeoSwerveConstants.SDSMK4i(driveGearRatios.SDSMK4i_L2);
        public static final COTSTalonFXSwerveConstants driveModule = COTSTalonFXSwerveConstants.WCP.SwerveXStandard.KrakenX60((6.75 / 1.0));

        /* Drivetrain Constants */
        public static final double trackWidth = Units.inchesToMeters(27); //TODO: This must be tuned to specific robot
        public static final double wheelBase = Units.inchesToMeters(21.5); //TODO: This must be tuned to specific robot
        public static final double wheelCircumference = chosenModule.wheelCircumference;

        public static final double driveRevToMeters =  wheelCircumference / (driveModule.driveGearRatio);
        public static final double driveRpmToMetersPerSecond = driveRevToMeters/60 ;

        /* Swerve Kinematics 
         * No need to ever change this unless you are not doing a traditional rectangular/square 4 module swerve */
         public static final SwerveDriveKinematics swerveKinematics = new SwerveDriveKinematics(
            new Translation2d(wheelBase / 2.0, trackWidth / 2.0),
            new Translation2d(wheelBase / 2.0, -trackWidth / 2.0),
            new Translation2d(-wheelBase / 2.0, trackWidth / 2.0),
            new Translation2d(-wheelBase / 2.0, -trackWidth / 2.0));

        /* Module Gear Ratios */
        public static final double driveGearRatio = driveModule.driveGearRatio;
        public static final double angleGearRatio = chosenModule.angleGearRatio;

         /* Motor Inverts */
        public static final boolean angleMotorInvert = chosenModule.angleMotorInvert;
        public static final InvertedValue driveMotorInvert = driveModule.driveMotorInvert;

        /* Angle Encoder Invert */
        public static final SensorDirectionValue cancoderInvert = chosenModule.cancoderInvert;

        /* Swerve Current Limiting */
        public static final int angleCurrentLimit = 25;
        public static final int angleCurrentThreshold = 40;
        public static final double angleCurrentThresholdTime = 0.1;
        public static final boolean angleEnableCurrentLimit = true;

        public static final int driveCurrentLimit = 40;
        public static final int driveCurrentThreshold = 60;
        public static final double driveCurrentThresholdTime = 0.1;
        public static final boolean driveEnableCurrentLimit = true;

        /* These values are used by the drive Motor to ramp in open loop and closed loop driving.
         * We found a small open loop ramp (0.25) helps with tread wear, tipping, etc */
        public static final double openLoopRamp = 0.2;
        public static final double closedLoopRamp = 0.0;

        /* Angle Motor PID Values */
        public static final double angleKP = 0.8;
        public static final double angleKI = 0.0;
        public static final double angleKD = 0.0;
        public static final double angleKF = 0.0;

        /* Drive Motor PID Values */
        public static final double driveKP = 0.05; //TODO: This must be tuned to specific robot
        public static final double driveKI = 0.0;
        public static final double driveKD = 0.0;
        public static final double driveKF = 0.0; //

         /* Heading PID Values */
        public static final double HeadingKP = 5.0;
        public static final double HeadingKI = 0.0;
        public static final double HeadingKD = 0;
        public static final double HeadingTolerence = 0;

        //Motor power gain
        public static final double drivePower = 1;
        public static final double anglePower = 0.9; //0.9


        /* Drive Motor Characterization Values from SysID */
        public static final double driveKS = (0.32); //TODO: This must be tuned to specific robot
        public static final double driveKV = (1.51);
        public static final double driveKA = (0.27);

        /* Swerve Profiling Values */
        /** Meters per Second */
        public static final double maxSpeed = 4.5; //TODO: This must be tuned to specific robot
        /** Radians per Second */
        public static final double maxAngularVelocity = 5.0; //TODO: This must be tuned to specific robot

        /* Neutral Modes */
        public static final NeutralModeValue talonAngleNeutralMode = NeutralModeValue.Coast;
        public static final  IdleMode angleNuetralMode = IdleMode.kCoast;
        public static final  IdleMode driveNuetralMode = IdleMode.kBrake;

        /* Module Specific Constants */
        /* Front Left Module - Module 0 */
        public static final class Mod3 { //TODO: This must be tuned to specific robot
            public static final int canCoderID = 1;
            public static final int driveMotorID = 2;
            public static final int angleMotorID = 3;
            public static final Rotation2d angleOffset = Rotation2d.fromRotations(0.171);
            public static final boolean driveInvert = true;
            public static final boolean angleInvert = false;
            public static final boolean angleMotorInvert = false;
            public static final double magnetOffset = 0;
            public static final SwerveModuleConstants constants = 
                new SwerveModuleConstants(
                    driveMotorID,
                    angleMotorID,
                    canCoderID,
                    angleOffset,
                    driveInvert,
                    angleInvert,
                    angleMotorInvert,
                    magnetOffset
                );
        }

        /* Front Right Module - Module 1 */
        public static final class Mod1 { //TODO: This must be tuned to specific robot
            public static final int canCoderID = 9;
            public static final int driveMotorID = 10;
            public static final int angleMotorID = 11;
            public static final Rotation2d angleOffset = Rotation2d.fromRotations(-0.052);
            public static final boolean driveInvert = true;
            public static final boolean angleInvert = false;
            public static final boolean angleMotorInvert = false;
            public static final double magnetOffset = 0;
            public static final SwerveModuleConstants constants = 
                new SwerveModuleConstants(
                    driveMotorID,
                    angleMotorID,
                    canCoderID,
                    angleOffset,
                    driveInvert,
                    angleInvert,
                    angleMotorInvert,
                    magnetOffset
                );

        }
        
        /* Back Left Module - Module 2 */
        public static final class Mod2 { //TODO: This must be tuned to specific robot
            public static final int canCoderID = 17;
            public static final int driveMotorID = 18;
            public static final int angleMotorID = 19;
            public static final Rotation2d angleOffset = Rotation2d.fromRotations(-0.266);
            public static final boolean driveInvert = false;
            public static final boolean angleInvert = false;
            public static final boolean angleMotorInvert = false;
            public static final double magnetOffset = 0;
            public static final SwerveModuleConstants constants = 
                new SwerveModuleConstants(
                    driveMotorID,
                    angleMotorID,
                    canCoderID,
                    angleOffset,
                    driveInvert,
                    angleInvert,
                    angleMotorInvert,
                    magnetOffset
                );
        }

        /* Back Right Module - Module 3 */
        public static final class Mod0 { //TODO: This must be tuned to specific robot
            public static final int canCoderID = 25;
            public static final int driveMotorID = 26;
            public static final int angleMotorID = 27;
            public static final Rotation2d angleOffset = Rotation2d.fromRotations(0.416);
            public static final boolean driveInvert = false;
            public static final boolean angleInvert = false;
            public static final boolean angleMotorInvert = false;
            public static final double magnetOffset = 0;
            public static final SwerveModuleConstants constants = 
                new SwerveModuleConstants(
                    driveMotorID,
                    angleMotorID,
                    canCoderID,
                    angleOffset,
                    driveInvert,
                    angleInvert,
                    angleMotorInvert,
                    magnetOffset
                );

        }
    }

    public static final class AutoConstants { //TODO: The below constants are used in the example auto, and must be tuned to specific robot
        public static final double kMaxSpeedMetersPerSecond = 3;
        public static final double kMaxAccelerationMetersPerSecondSquared = 3;
        public static final double kMaxAngularSpeedRadiansPerSecond = Math.PI;
        public static final double kMaxAngularSpeedRadiansPerSecondSquared = Math.PI;

        //TODO: Must be tuned to specific robot
        public static final PIDConstants translationPID = new PIDConstants(0.5, 0, 0);
        public static final PIDConstants rotationPID = new PIDConstants(0.8, 0, 0);

        //TODO: Must be tuned to specific robot
        public static final double ROBOT_MASS_KG = 20.000000000001;
        public static final double ROBOT_MOI = 6.883;
        public static final double WHEEL_COF = 1.2;

        public static final ModuleConfig moduleConfig = new ModuleConfig(
                (Constants.Swerve.chosenModule.wheelDiameter / 2),
                (Constants.Swerve.maxSpeed),
                Constants.AutoConstants.WHEEL_COF,
                DCMotor.getNEO(1).withReduction(Constants.Swerve.chosenModule.driveGearRatio),
                Constants.Swerve.driveCurrentThreshold,
              1);
    
        /* Constraint for the motion profilied robot angle controller */
        public static final TrapezoidProfile.Constraints kThetaControllerConstraints =
            new TrapezoidProfile.Constraints(
                kMaxAngularSpeedRadiansPerSecond, kMaxAngularSpeedRadiansPerSecondSquared);
    }

    public static final class PoseEstimator {
        public static final Matrix<N3,N1> stateStdDevs = VecBuilder.fill(0.1, 0.1, 0.1);
        public static final Matrix<N3,N1> visionStdDevs = VecBuilder.fill(0.9, 0.9, 0.9);
    }

    public static final class CoralOutIntake{
        public static final int JOYSTICK_PORT_ID = 0;

        public static final int TOP_MOTOR_ID = 61;
        public static final int BOTTOM_MOTOR_ID = 62;


        //The following lines of code are needed to be tested to determine the setponts {//
            public static final double HIGH_POSITION = 0;
            public static final double MID_POSITION = 0;
            public static final double LOW_POSITION = 0;
        //}

        public static final double ROTATION_MOTOR_SPEED = 1;

        public static final double Kp = 0;
        public static final double Ki = 0;
        public static final double Kd = 0;
    }

    public class Elevator {
        public static final int LEFT_MOTOR_ID = 9;
        public static final int RIGHT_MOTOR_ID = 59;
        public static final double ERROR_TOLERANCE = 0.1;
        public static final double MAX = 197.0; 
        public static final double MIN = 7;
        // get the position for the following //{
        //public static final double L1_POSITION = 5;
        //public static final double L2_POSITION = 42.5;
        //public static final double L3_POSITION = 90;
        //public static final double L4_POSITION = 130;
        public static final double L1_POSITION = 0;
        public static final double L2_POSITION = 10;
        public static final double L3_POSITION = 20;
        public static final double L4_POSITION = 55;
        //}

        public static final double KP = 0.5;
        public static final double KI = 0.01;
        public static final double KD = 0;

        public static final boolean MOTORINVERT = false;
        public static final boolean STEERINVERT = false;
        public static final double NEUTRALMODE = 0.0;
        public static final double GEARRATIO = 1;
        public static final boolean ENABLECURRENTLIMIT = true;
        public static final double CURRENTLIMIT = 40;
        public static final double OPENLOOPRAMP = 0.2;
        public static final double CLOSEDLOOPRAMP = 0.0;
    }

    public static final class Climb {

    }

    public static final class LEDs {
        public static final int LED_BUFFER_LENGTH = 0;
        public static final int LED_PORT = 0;
        public static final Distance kLED_SPACING = Meters.of(1 / 120.0); // One meter for a certain amount of LEDs.
        /* LED PATTERNS FOR EASY USAGE */
        public static final LEDPattern[] LED_PATTERNS = {
            LEDPattern.solid(Color.kRed), //Alliance Red
            LEDPattern.solid(Color.kBlue), // Alliance Blue
            LEDPattern.solid(Color.kGreen), // Deep Cage
            LEDPattern.solid(Color.kAquamarine), //Elevator
            LEDPattern.solid(Color.kHotPink), //Algae
            LEDPattern.solid(Color.kYellow), // Coral
            LEDPattern.solid(Color.kDarkOrange) // PIT MODE
        };

    }


}

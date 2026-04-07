package frc.robot;

import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.pathplanner.lib.config.PIDConstants;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.util.Units;
import frc.lib.util.CANSparkMaxUtil.Usage;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

public final class Constants {
    /* Used for Constants Used Once On Initialization of Robot or Subsystems */
    public final static class Setup {
        /* Swerve Module Ids and Constants */
        public static final int[] moduleIDs = new int[] {0, 1, 2, 3};
        public static final int[] driveMotors = new int[] {1, 3, 5, 7};
        public static final int[] angleMotors = new int[] {2, 4, 6, 8};
        public static final int[] moduleCancoders = new int[] {9, 10, 11, 12};
        public static final double[] angleOffsets = new double[] {-151.5, 24.0, 1.0, 122.5};
        public static final double gyroAngleOffset = 0.0; // If gyro is mounted at an angle, this should tell the robot which way is forward

        /* Subsystem IDs */
        public static final int hopperSpark = 14;
        public static final int intakeSpark = 15;
        public static final int floorSpark = 16;
        public static final int feederSpark = 17;

        public static final int[] shooterSpark = new int[] {18, 19, 20};

        /* Motor Inverts */
        public static final boolean driveInvert = true;
        public static final boolean angleInvert = true;

        public static final boolean hopperInvert = false;
        public static final boolean intakeInvert = true;
        public static final boolean floorInvert = true;
        public static final boolean feederInvert = false;
        public static final boolean shooterInvert = true;
    }

    /* Autonomous config
     * This is here to keep track of which autos we want pasted straight in from PathPlanner. If an auto
     * is on the supportAutoList, it is IGNORED and not added to the autolist. These should be autos that
     * are only used in compound autonomous routines.
     */
    public record Auto (String[] supportAutoList){}
    public static Auto AutoConfig = new Auto(new String[]{"Example Supp Auto", "Back Off Reef"});

    public final static class Swerve {
        public static final double stickDeadband = 0.1;
        public static final double autoAimTolerance = 1.0;

        /* Drivetrain Calculation Constants */
        /* Input these units from center of swerve modules */
        public static final double trackWidth = Units.inchesToMeters(26.0);
        public static final double trackLength = Units.inchesToMeters(28.0);

        /* Input Current Wheel Diameter, Can Change Due To Amount Of Wear */
        // Tread replaced 1/31/26
        public static final double wheelDiameter = Units.inchesToMeters(4); // Wheel diameter in inches
        public static final double wheelCircumference = wheelDiameter * Math.PI;

        /* Gyro Direction Toggle */
        public static final boolean invertGyro = true; // Always ensure Gyro is CCW+ CW- (Clockwise is increasing rotation values)

        /* Cancoder Invert */
        public static final boolean canCoderInvert = false;

        /* Speed Settings */
        public static final double maxSpeed = 5.00; // meters per second
        public static final double maxAngularVelocity = 7; // radians per second (was 4.25, changed because turn speed suddenly dropped)

        /* Mk4i Module Gear Ratios */
        public static final double driveGearRatio = (6.75 / 1.0); // 6.75:1
        public static final double angleGearRatio = (150.0 / 7.0); // 150:7
    

        /* Swerve Module Positions (Currently in solid rectangle context) */
        public static final Translation2d[] modulePositions = new Translation2d[] {
            new Translation2d( (trackLength / 2.0) - Units.inchesToMeters(2.635),  (trackWidth / 2.0) - Units.inchesToMeters(2.635)),
            new Translation2d( (trackLength / 2.0) - Units.inchesToMeters(2.635), (-trackWidth / 2.0) + Units.inchesToMeters(2.635)),
            new Translation2d((-trackLength / 2.0) + Units.inchesToMeters(2.635),  (trackWidth / 2.0) - Units.inchesToMeters(2.635)),
            new Translation2d((-trackLength / 2.0) + Units.inchesToMeters(2.635), (-trackWidth / 2.0) + Units.inchesToMeters(2.635))
        };

        /* Swerve Kinematics */
        public static final SwerveDriveKinematics swerveKinematics =
        new SwerveDriveKinematics(
            modulePositions[0],
            modulePositions[1],
            modulePositions[2],
            modulePositions[3]
        );

        public static final PPHolonomicDriveController pathFollowerConfig = new PPHolonomicDriveController(
            new PIDConstants(3, 0, 0.2), // Translation constants (should be in volts/meter of error)
            new PIDConstants(0.5, 0, 0) // Rotation constants
            // 2024 -> 2025 import change. Constructor simplified, deleted maxspeed, drive base radius, and replanning config
        );
    }

    public static final class Hopper {
        /* Min/Max Speeds */
        public static final double maxVoltage = 5.0;
        public static final double maxPercent = 0.55;    // | Power limits for PID control
        public static final double minPercent = -0.40;   // |

        /* Min/Max Heights */
        public static final double minAngle = 0.0;       // Should be in degrees. Starting point will be 0
        public static final double maxAngle = 103.0;
        
        /* Tolerance */
        public static final double tolerance = 1.0;
    }

    public static final class Shooter {
        public static final double delay = 0.7;
        public static final double voltFactor = 0.492;
        public static final double flatVolts = 0.25;
        public static final int lostFramesTolerance = 15;
    }

    public static final class AutoConstants {
        public static final double kPXController = 1;
        public static final double kPYController = 1;
        public static final double kPThetaController = 1;
    }

    public final static class Electrical {
        /* Base 12 Volt System */
        public static final double voltageComp = 12.0;

        /* Swerve Electrical Limits */
        public static final int driveCurrentLim = 40;
        public static final int angleCurrentLim = 20;
        
        /* Subsystems */
        public static final int hopperLim = 20;
        public static final int intakeLim = 40;
        public static final int floorLim = 30;
        public static final int feederLim = 40;
        public static final int shooterLim = 40;
    }
    
    public final static class PID {
        /* Format {P, I, D, FF} 
        P, I, and D are standard PID
        FF is the inverse of kV in SVA control. Used ONLY in velocity control.
        */

        /* Swerve PIDs */
        public static final double[] drivePID = new double[] {0.05, 0.0, 0.0, 0.0};
        public static final double[] anglePID = new double[] {10.0, 1.1, 0.2, 0.0};
        
        /* Subsystems */
        public static final double[] hopperPID = new double[] {0.017, 0.0, 0.0, 0.0};   // TODO - potentially add a little I or D? Not sure yet, will want testing
        public static final double[] intakePID = new double[] {0.05, 0.0, 0.0, 0.0};
        public static final double[] floorPID = new double[] {0.03, 0.0, 0.0, 0.0};
        public static final double[] feederPID = new double[] {0.03, 0.0, 0.0, 0.0};
        public static final double[] shooterPID = new double[] {0.03, 0.0, 0.0, 0.0};
    }

    public final static class SVA {
        /* {Static, Velocity, Acceleration} */    /* format: Ks, Kv, Ka */
        /* Swerve */
        public static final double[] driveMotorsSVA = new double[] {0.24, 2.4, 0.18};   // IN PROGRESS - SysID tuning.
    }

    public final static class ConversionFactors {
        /* All numbers in 1 output to required input, or one wheel spin to motor spin */

        /* Swerve Drive Conversions */
        public static final double driveKrakenRotorToSensorRatio = 1.0;
        public static final double driveKrakenSensorToMechanismRatio = Swerve.driveGearRatio / Swerve.wheelCircumference;   // switched around for krakens
        
        // This should work but it won't give us degrees
        public static final double angleKrakenRotorToSensorRatio = 1;
        public static final double angleKrakenSensorToMechanismRatio = Swerve.angleGearRatio;
        
        /* Other Subsystem Conversions */
        public static final double hopperConversionPositionFactor = 1.0/115.0 * 360.0;    // 1:115 gear reduction, 1:360 degree conversion
        public static final double hopperConversionVelocityFactor = hopperConversionPositionFactor / 60.0; //rpm to rps
        
        // 11.0/18.0 * 0.318;   <---   Old one (for reference)
        public static final double shooterConversionPositionFactor = 13.0/16.0 * 0.318;    // 13:16 gear reduction, 4in diameter * pi conversion to circumference
        public static final double shooterConversionVelocityFactor = shooterConversionPositionFactor / 60.0; //rpm to rps

        /* These just get the raw encoder readings */
        public static final double defaultConversionPositionFactor = 1.0;
        public static final double defaultConversionVelocityFactor = defaultConversionPositionFactor / 60.0; //rpm to rps
    }

    public final static class IdleModes {
        /* Swerve Idles */
        public static final NeutralModeValue driveIdle = NeutralModeValue.Brake;
        public static final NeutralModeValue angleIdle = NeutralModeValue.Brake;

        public static final NeutralModeValue intakeIdle = NeutralModeValue.Brake;

        public static final IdleMode hopperIdle = IdleMode.kBrake;
        public static final IdleMode floorIdle = IdleMode.kCoast;
        public static final IdleMode feederIdle = IdleMode.kCoast;
        public static final IdleMode shooterIdle = IdleMode.kCoast;
    }

    public final static class Usages {
        /* How the device should utilize the CAN connection
         * Options: kAll - standard high CAN data publishing setting
         *          kPositionOnly - publish motor position often, velocity rarely
         *          kVelocityOnly - publish motor velocity often, position rarely
         *          kMinimal - useful for motors we need no/little feedback from
         */
        /* Swerve Usages */
        public static final Usage driveUsage = Usage.kAll;
        public static final Usage angleUsage = Usage.kPositionOnly;

        public static final Usage hopperUsage = Usage.kPositionOnly;
        public static final Usage intakeUsage = Usage.kVelocityOnly;
        public static final Usage floorUsage = Usage.kMinimal;
        public static final Usage feederUsage = Usage.kMinimal;
        public static final Usage shooterUsage = Usage.kVelocityOnly;
    }

    public final static class Limelight {
        public static final double maxRate = 40.0;  // slew rate, in degrees/s
    }
}
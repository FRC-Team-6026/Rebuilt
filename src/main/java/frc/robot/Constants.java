package frc.robot;

import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.pathplanner.lib.config.PIDConstants;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.util.Units;
import frc.lib.util.CANSparkMaxUtil.Usage;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.ctre.phoenix6.signals.NeutralModeValue;

public final class Constants {
    /* Used for Constants Used Once On Initialization of Robot or Subsystems */
    public final static class Setup {

        /* Swerve Module Ids and Constants */
        public static final int[] moduleIDs = new int[] {0, 1, 2, 3};
        public static final int[] driveMotors = new int[] {1, 3, 5, 7};
        public static final int[] angleMotors = new int[] {2, 4, 6, 8};
        public static final int[] moduleCancoders = new int[] {9, 10, 11, 12};
        public static final double[] angleOffsets = new double[] {-132.5, 143.0, 55.5, -137.5};
        public static final double gyroAngleOffset = 0.0; // If gyro is mounted at an angle, set this to fix it. Actually, I'm not sure this fixes that...

        /* Intake IDs */
        public static final int wristSpark = 16;

        public static final int clawSpark = 17;

        public static final int elevatorSpark1 = 18;
        public static final int elevatorSpark2 = 19;

        public static final int beambreakID = 0;
        // public static final int physicalSwitchID = 2;

        /* Motor Inverts */
        public static final boolean driveInvert = false;
        public static final boolean angleInvert = true;

        public static final boolean clawInvert = false;
        public static final boolean elevatorInvert = false;
        public static final boolean wristInvert = true;


        public enum shooterInverts {
            left(true),
            right(false);
            public final boolean Invert;
            shooterInverts(boolean Invert){
                this.Invert = Invert;
            }
        }
    }

    /* Autonomous config
     * This is here to keep track of which autos we want pasted straight in from PathPlanner. If an auto
     * is on the list, it is IGNORED and not added to the autolist. These should be autos that are used
     * in compound autonomous routines.
     */
    public record Auto ( String defaultAutoName, String[] supportAutoList){}
    public static Auto AutoConfig = new Auto("", new String[]{"Example Supp Auto", "Back Off Reef"});

    public final static class Swerve {
        public static final double stickDeadband = 0.1;
        public static final double autoAimTolerance = 1.0;

        /* Drivetrain Calculation Constants */
        /* Input these units from center of swerve modules */
        public static final double trackWidth = Units.inchesToMeters(26.0);
        public static final double trackLength = Units.inchesToMeters(28.0);

        /* Input Current Wheel Diameter, Can Change Due To Amount Of Wear */
        // TODO - change this back to 4 if we replace wheel tread.
        public static final double wheelDiameter = Units.inchesToMeters(3.9); // Wheel diameter in inches
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

        // TODO - rotation P was 1.0, testing lower value. if the problem was something else, probably put it back.
        // Values moved down below with other PID values to keep everything together
        public static final PPHolonomicDriveController pathFollowerConfig = new PPHolonomicDriveController(
            new PIDConstants(2, 0, 0.2), // Translation constants (should be in volts/meter of error)
            new PIDConstants(0.5, 0, 0) // Rotation constants
            // 2024 -> 2025 import change. Constructor simplified, deleted maxspeed, drive base radius, and replanning config
        );
    }

    public static enum Level {Retracted, Processor, L1, L2, L2A, L3, L3A, L4, Barge}
    public static enum Location {ReefLeft, ReefRight, ReefCenter, Pickup, Processor}
    
    public static final class Claw {

        // gear ratio moved to conversion factors

        /* Min/Max Speeds */
        public static final double intakeSpeed = 5;
        public static final double maxVoltage = 5;

    }

    public static final class Elevator {

        // gear ratio moved to conversion factors

        /* Min/Max Speeds */
        public static final double maxVoltage = 3.0;
        public static final double minVoltage = -1.8;
        public static final double maxPercent = 0.55;    // | Power limits for PID control
        public static final double minPercent = -0.40;   // |

        /* Min/Max Heights */
        public static final double minHeight = 0.0;       // Now in inches!
        public static final double maxHeight = 59.0;
        
        /* setElevator height seeking tolerance */
        public static final double tolerance = 0.4; // Rename this pls

        public static final double selfDestructAngle = 61;

        // public static final double gravityConstant = 0.1;

        public static final double softHeightMinimum = 1;
    }

    public static final class AutoConstants {
        
        public static final double kPXController = 1;
        public static final double kPYController = 1;
        public static final double kPThetaController = 1;
    
    }

    public static final class Wrist{

        public static final double minimumAngle = 41.0;  // Coral Intake Position
        public static final double maximumAngle = 220.0; // 
        public static final double alignmentAngle = 64.0;

        public static final double L23ScoringAngle = 67.0;
        public static final double L4ScoringAngle = 84.0;
        public static final double algaePickupAngle = 195.0;
        public static final double processorAngle = 210.0;
        public static final double bargeSetup = 150.0;
        public static final double bargeAngle = 100.0;

        // Gear reduction moved to conversion factors
        
        public static final double maxVoltage = 3.0;
        public static final double maxPercent = 0.5;    // | Power limits for PID control
        public static final double minPercent = -0.5;   // |

        // public static final double maxAccel = 180.0;
        // public static final double maxSpeed = 90.0;

        public static final double angleTolerance = 1.2;
        public static final double gearReduction = 0.0;
    }

    public final static class Electrical {

        /* Base 12 Volt System */
        public static final double voltageComp = 12.0;

        /* Swerve Electrical Limits */
        public static final int driveCurrentLim = 40;
        public static final int angleCurrentLim = 20;
        
        /* Subsystems */
        public static final int clawLim = 20;
        public static final int elevatorLim = 40;
        public static final int wristLim = 20;

        public static final double neoMinVoltage = 0.05; // guessed value
    }
    
    public final static class PID {

        /* Format {P, I, D, FF} 
        P, I, and D are standard PID
        FF is the inverse of kV in SVA control. Used ONLY in velocity control.
        */

        /* Swerve PIDs */
        public static final double[] drivePID = new double[] {0.1, 0.0, 0.0, 0.0};   // TODO - After testing, change back to {0.3, 0.0, 0.0, 0.0}
        public static final double[] anglePID = new double[] {1.2, 0.0, 0.0, 0.0};  // TODO - After testing, change back to {0.01, 0.0, 0.0, 0.0}
        
        /* Subsystems */
        public static final double[] clawPID = new double[] {0.05, 0.0, 0.0, 0.0};
        public static final double[] elevatorPID = new double[] {0.032, 0.0003, 0.006, 0.0};
        public static final double[] wristPID = new double[] {0.023, 0.0, 0.025, 0.0};
    }

    public final static class SVA {

        /* {Static, Velocity, Acceleration} */    /* format: Ks, Kv, Ka */
        /* Swerve */
        // public static final double[] driveMotorsSVA = new double[] {0.3, 2.55, 0.27};    // 2023's SVA values. 
        public static final double[] driveMotorsSVA = new double[] {0.24, 2.60, 0.46};

        // public static final double[] ElevSVA = new double[] {0.0, 0.2, 0.00};
        // public static final double[] WristSVA = new double[] {0.05, 0.03, 0.001};
    }

    public final static class ConversionFactors {
        /* All numbers in 1 output to required input, or one wheel spin to motor spin */

        /* Swerve Drive Conversions */
        //Sparkmax constants
        public static final double driveConversionPositionFactor = Swerve.wheelCircumference / Swerve.driveGearRatio;
        public static final double driveConversionVelocityFactor = driveConversionPositionFactor / 60.0; //rpm to rps (sparkmax only, not kraken)
        
        public static final double angleConversionPositionFactor = 360.0 / Swerve.angleGearRatio;
        public static final double angleConversionVelocityFactor = angleConversionPositionFactor / 60.0; //rpm to rps

        //Kraken constants
        public static final double driveKrakenRotorToSensorRatio = 1.0;
        public static final double driveKrakenSensorToMechanismRatio = Swerve.driveGearRatio / Swerve.wheelCircumference;   // switched around for krakens
        
        // This should work but it won't give us degrees
        public static final double angleKrakenRotorToSensorRatio = 1;
        public static final double angleKrakenSensorToMechanismRatio = Swerve.angleGearRatio;

        // This should work with our current setup, i think. does NOT work with the built-in continuousWrap
        // public static final double angleKrakenRotorToSensorRatio = 360.0;
        // public static final double angleKrakenSensorToMechanismRatio = Swerve.angleGearRatio / 360.0;
        
        /* Other Subsystem Conversions */
        // public static final double elevatorConversionPositionFactor = 1/6.4;   // 10 tooth small : 64 tooth large
        public static final double elevatorConversionPositionFactor = 1.406;   // 10 tooth small : 64 tooth large, 1 rot : 3.25 in of chain (18 tooth sprocket), 1 inch of 1st stage : 2 inch of 2nd stage
        public static final double elevatorConversionVelocityFactor = elevatorConversionPositionFactor / 60.0; //rpm to rps
        
        public static final double wristConversionPositionFactor = 1.0/96.0 * 360;   // 96:1 total gear reduction, 1:360 degree conversion
        public static final double wristConversionVelocityFactor = elevatorConversionPositionFactor / 60.0; //rpm to rps
        
        /* These just get the raw encoder readings */
        public static final double defaultConversionPositionFactor = 1.0;
        public static final double defaultConversionVelocityFactor = defaultConversionPositionFactor / 60.0; //rpm to rps

    }

    public final static class IdleModes {
        /* Swerve Idles */
        public static final IdleMode driveIdle = IdleMode.kBrake;
        public static final IdleMode angleIdle = IdleMode.kBrake;
        public static final NeutralModeValue krakenDriveIdle = NeutralModeValue.Brake;
        public static final NeutralModeValue krakenAngleIdle = NeutralModeValue.Coast;

        public static final IdleMode clawIdle = IdleMode.kBrake;
        public static final IdleMode elevatorIdle = IdleMode.kBrake;
        public static final IdleMode wristIdle = IdleMode.kBrake;
    }

    public final static class Usages {
        /* Swerve Usages */
        public static final Usage driveUsage = Usage.kAll;
        public static final Usage angleUsage = Usage.kPositionOnly;

        public static final Usage clawUsage = Usage.kPositionOnly;
        public static final Usage elevatorUsage = Usage.kPositionOnly;
        public static final Usage wristUsage = Usage.kPositionOnly;
    }
}
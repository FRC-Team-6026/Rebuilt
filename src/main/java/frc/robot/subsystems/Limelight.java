package frc.robot.subsystems;

import java.util.Map;
import java.util.concurrent.ConcurrentHashMap;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.networktables.DoubleArrayEntry;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.TimestampedDoubleArray;
import edu.wpi.first.wpilibj.Preferences;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

/* Coords from the perspective of the camera
 *  / Z (pointing out)
 * +-----> X 
 * |
 * |
 * V Y
 * 
 * Yaw: 0d is facing red alliance
 *    ____
 *   /    \
 *   V    V
 * CCW+  CW-
 */



public class Limelight extends SubsystemBase {
    private final NetworkTableInstance _instance = NetworkTableInstance.getDefault();
    private NetworkTable _table;
    public double fieldRot; // degree angle to add to the gyro yaw to get our angle on the field.
    public boolean periodicRotationUpdate = false;
    public Swerve swerve;
    public SlewRateLimiter limiter;

    public Limelight(String networkTableName, Swerve swerve) {
        _table = _instance.getTable(networkTableName);
        fieldRot = 0.0;
        this.swerve = swerve;
        this.limiter = new SlewRateLimiter(Constants.Limelight.maxRate);
    }

    @Override
    public void periodic() {
        if (periodicRotationUpdate) {
            SetRobotOrientation("limelight", swerve.getGyro().getYaw() + fieldRot, 0, 0, 0, 0, 0);
        }
    }

    public boolean isTargets() {
        return _table.getEntry("tv").getInteger(0) > 0.5;
    }

    // TODO - Reprogram Autoaim
    /** Gets L/R movement for robot to align with an apriltag */
    public double getTX() {
        double tx = _table.getEntry("tx").getDouble(0) * -1.0;
        tx = limiter.calculate(tx);
        SmartDashboard.putNumber("tx", tx);
        if (tx > 0.5 || tx < -0.5)
            return Math.signum(tx)*0.02 + tx*Preferences.getDouble("AutoDriveStrength", 1.0)/100.0;
        return 0.0;
    }

    /** Gets L/R movement for robot to align with an apriltag 
     * @param targetOffset the point to target, relative to the center of the image
    */
    public double getTX(double targetOffset) {
        double tx = _table.getEntry("tx").getDouble(0) * -1.0;
        tx = limiter.calculate(tx);
        SmartDashboard.putNumber("tx", tx);
        tx += targetOffset;
        if (tx > 0.5 || tx < -0.5)
            return Math.signum(tx)*0.02 + tx*Preferences.getDouble("AutoDriveStrength", 1.0)/100.0;
        return 0.0;
    }

    /** Gets F/B movement for robot to align with an apriltag */
    public double getTZ() {
        double tz = _table.getEntry("camerapose_targetspace").getDoubleArray(new double[6])[2] * -1.0;
        SmartDashboard.putNumber("tz", tz);
        if (tz > 0.25)
            return Math.signum(tz)*0.25 + tz*Preferences.getDouble("AutoDriveStrength", 1.0)/10.0;
        return 0.04;
    }
    /*
    public double getTZ() {
        double tz = _table.getEntry("camerapose_targetspace").getDoubleArray(new double[6])[2] * -1.0;
        SmartDashboard.putNumber("tz", tz);
        if (tz > 0.3)
            return Math.signum(tz)*0.2 + tz*Preferences.getDouble("AutoDriveStrength", 1.0)/8.0;
        return 0.0;
    }
     */

    /** This function should set the offset between the rotation from
     * our gyro and the rotation in field space.
     */
    public boolean configRotation(Swerve swerve) {
        SmartDashboard.putNumber("Before Config pose rot:", Math.round(swerve.getPose().getRotation().getDegrees()));
        SmartDashboard.putNumber("Before Config gyro:", Math.round(swerve.getGyro().getYaw()));
        PoseEstimate limelightMeasurement;
        boolean doRejectUpdate = false;

        limelightMeasurement = getBotPoseEstimate("limelight", "botpose_wpiblue", false);
        
        if (limelightMeasurement.tagCount == 0)
            doRejectUpdate = true;
        if (limelightMeasurement.tagCount == 1) {
            if (limelightMeasurement.rawFiducials[0].ambiguity > .7)
                doRejectUpdate = true;
            if (limelightMeasurement.rawFiducials[0].distToCamera > 3)
                doRejectUpdate = true;
        }
        
        if (doRejectUpdate)
            return false;
        
        fieldRot = limelightMeasurement.pose.getRotation().getDegrees() - (swerve.getGyro().getYaw());
        SmartDashboard.putNumber("pose est rot", limelightMeasurement.pose.getRotation().getDegrees());
        SmartDashboard.putNumber("gyro yaw", swerve.getGyro().getYaw());
        SmartDashboard.putNumber("field rotation offset", fieldRot);

        periodicRotationUpdate = true;

        return true;
    }

    /** This function directly sets the offset between the rotation from
     * our gyro and the rotation in field space.
     */
    public boolean configRotation(Rotation2d rotation) {
        return configRotation(rotation.getDegrees());
    }

    /** This function directly sets the offset between the rotation from
     * our gyro and the rotation in field space.
     */
    public boolean configRotation(double degrees) {
        fieldRot = degrees;
        SmartDashboard.putNumber("field rotation offset", fieldRot);
        return true;
    }

    public int getTagID() {
        return (int) _table.getEntry("tid").getInteger(-1);
    }
    
    public Pose2d getRobotPoseInTargetSpace() {
        double[] robotPoseArray = new double[6];
        robotPoseArray = _table.getEntry("botpose_targetspace").getDoubleArray(robotPoseArray);
        double x = robotPoseArray[0];
        double z = robotPoseArray[2];
        double rotation = robotPoseArray[4];

        return new Pose2d(x, z, Rotation2d.fromDegrees(rotation));
    }

    /**
     * Uses AprilTags in view to update the robot's position, with Limelight's MegaTag (or MegaTag 2).
     * If the pose is not likely accurate, this function does nothing. Otherwise, it updates the pose.
     * 
     * @param swerve the swerve system to send the updated pose to
     * @return true if update was successful, otherwise false
     */
    public boolean updatePose(Swerve swerve, boolean megatag2) {
        Pose2d pose = swerve.getPose();
        SmartDashboard.putString("Pose before update", "("+
            Math.round(pose.getX()*100)/100.0 + ", " +
            Math.round(pose.getY()*100)/100.0 + ", " +
            Math.round(pose.getRotation().getDegrees()) + "d)"
        );
        double yaw = swerve.getGyro().getYaw();
        PoseEstimate limelightMeasurement;
        boolean doRejectUpdate = false;

        if (megatag2) {
            SmartDashboard.putString("Did we use MT2", "Yes");
            SetRobotOrientation("limelight", yaw + fieldRot, 0, 0, 0, 0, 0);
            // Try this?                                   limelightMeasurement.pose.getRotation().getDegrees()
            
            limelightMeasurement = getBotPoseEstimate("limelight", "botpose_wpiblue", true);

            if (Math.abs(swerve.getGyro().getRate()) > 720) // if our angular velocity is greater than 720 degrees per second, ignore vision updates
                { doRejectUpdate = true; }
            if (limelightMeasurement.tagCount == 0)
                { doRejectUpdate = true; }
        }
        else {
            limelightMeasurement = getBotPoseEstimate("limelight", "botpose_wpiblue", false);
            SmartDashboard.putString("Did we use MT2", "No");
            if (limelightMeasurement.tagCount == 0)
                doRejectUpdate = true;
            if (limelightMeasurement.tagCount == 1) {
                if (limelightMeasurement.rawFiducials[0].ambiguity > .7)
                    doRejectUpdate = true;
                if (limelightMeasurement.rawFiducials[0].distToCamera > 3)
                    doRejectUpdate = true;
            }
        }
        if (Math.abs(swerve.getPose().getX() - limelightMeasurement.pose.getX()) > 0.8 ||
            Math.abs(swerve.getPose().getY() - limelightMeasurement.pose.getY()) > 0.8 ||
            Math.abs(swerve.getPose().getRotation().getDegrees() - limelightMeasurement.pose.getRotation().getDegrees()) > 90)
        {
            doRejectUpdate = true;
            SmartDashboard.putString("UpdatePose estimate off?", "yes");
        }
            
        
        if (!doRejectUpdate)
        {
            swerve.resetOdometry(limelightMeasurement.pose);
        }

        pose = limelightMeasurement.pose;
        SmartDashboard.putString("Pose after update", "("+
            Math.round(pose.getX()*100)/100.0 + ", " +
            Math.round(pose.getY()*100)/100.0 + ", " +
            Math.round(pose.getRotation().getDegrees()*100)/100.0 + "d)"
        );

        return !doRejectUpdate;
    }

    

    /* Everything from this line down is copied over or lightly edited from the LimelightHelpers class provided by Limelight. 
     * We shouldn't need to interact with most of it except for the SetRobotOrientation and getBotPoseEstimate methods.
     */
    
    
     

     private static final Map<String, DoubleArrayEntry> doubleArrayEntries = new ConcurrentHashMap<>();

     /**
      * Represents a Limelight Raw Fiducial result from Limelight's NetworkTables output.
      */
     public static class RawFiducial {
         public int id = 0;
         public double txnc = 0;
         public double tync = 0;
         public double ta = 0;
         public double distToCamera = 0;
         public double distToRobot = 0;
         public double ambiguity = 0;
 
 
         public RawFiducial(int id, double txnc, double tync, double ta, double distToCamera, double distToRobot, double ambiguity) {
             this.id = id;
             this.txnc = txnc;
             this.tync = tync;
             this.ta = ta;
             this.distToCamera = distToCamera;
             this.distToRobot = distToRobot;
             this.ambiguity = ambiguity;
         }
     }
 
      
     /**
      * Represents a 3D Pose Estimate.
      */
     public static class PoseEstimate {
         public Pose2d pose;
         public double timestampSeconds;
         public double latency;
         public int tagCount;
         public double tagSpan;
         public double avgTagDist;
         public double avgTagArea;
 
         public RawFiducial[] rawFiducials; 
         public boolean isMegaTag2;
 
         /**
          * Instantiates a PoseEstimate object with default values
          */
         public PoseEstimate() {
             this.pose = new Pose2d();
             this.timestampSeconds = 0;
             this.latency = 0;
             this.tagCount = 0;
             this.tagSpan = 0;
             this.avgTagDist = 0;
             this.avgTagArea = 0;
             this.rawFiducials = new RawFiducial[]{};
             this.isMegaTag2 = false;
         }
 
         public PoseEstimate(Pose2d pose, double timestampSeconds, double latency, 
             int tagCount, double tagSpan, double avgTagDist, 
             double avgTagArea, RawFiducial[] rawFiducials, boolean isMegaTag2) {
 
             this.pose = pose;
             this.timestampSeconds = timestampSeconds;
             this.latency = latency;
             this.tagCount = tagCount;
             this.tagSpan = tagSpan;
             this.avgTagDist = avgTagDist;
             this.avgTagArea = avgTagArea;
             this.rawFiducials = rawFiducials;
             this.isMegaTag2 = isMegaTag2;
         }
 
     }
 
     /**
      * Takes a 6-length array of pose data and converts it to a Pose2d object.
      * Uses only x, y, and yaw components, ignoring z, roll, and pitch.
      * Array format: [x, y, z, roll, pitch, yaw] where angles are in degrees.
      * @param inData Array containing pose data [x, y, z, roll, pitch, yaw]
      * @return Pose2d object representing the pose, or empty Pose2d if invalid data
      */
     public static Pose2d toPose2D(double[] inData){
         if(inData.length < 6)
         {
             //System.err.println("Bad LL 2D Pose Data!");
             return new Pose2d();
         }
         Translation2d tran2d = new Translation2d(inData[0], inData[1]);
         Rotation2d r2d = new Rotation2d(Units.degreesToRadians(inData[5]));
        
         return new Pose2d(tran2d, r2d);
     }
 
     static final String sanitizeName(String name) {
         if (name == "" || name == null) {
             return "limelight";
         }
         return name;
     }
 
     public static void Flush() {
         NetworkTableInstance.getDefault().flush();
     }
 
     public static NetworkTableEntry getLimelightNTTableEntry(String tableName, String entryName) {
         return getLimelightNTTable(tableName).getEntry(entryName);
     }
 
     public static NetworkTable getLimelightNTTable(String tableName) {
         return NetworkTableInstance.getDefault().getTable(sanitizeName(tableName));
     }
 
     public static DoubleArrayEntry getLimelightDoubleArrayEntry(String tableName, String entryName) {
         String key = tableName + "/" + entryName;
         return doubleArrayEntries.computeIfAbsent(key, k -> {
             NetworkTable table = getLimelightNTTable(tableName);
             return table.getDoubleArrayTopic(entryName).getEntry(new double[0]);
         });
     }
 
     private static double extractArrayEntry(double[] inData, int position){
         if(inData.length < position+1) { return 0; }
         return inData[position];
     }
 
 
     /**
      * Sets robot orientation values used by MegaTag2 localization algorithm.
      * 
      * @param limelightName Name/identifier of the Limelight
      * @param yaw Robot yaw in degrees. 0 = robot facing red alliance wall in FRC
      * @param yawRate (Unnecessary) Angular velocity of robot yaw in degrees per second
      * @param pitch (Unnecessary) Robot pitch in degrees 
      * @param pitchRate (Unnecessary) Angular velocity of robot pitch in degrees per second
      * @param roll (Unnecessary) Robot roll in degrees
      * @param rollRate (Unnecessary) Angular velocity of robot roll in degrees per second
      */
     public static void SetRobotOrientation(String limelightName, double yaw, double yawRate, 
         double pitch, double pitchRate, 
         double roll, double rollRate) {
         SetRobotOrientation_INTERNAL(limelightName, yaw, yawRate, pitch, pitchRate, roll, rollRate, true);
     }
 
     public static void SetRobotOrientation_NoFlush(String limelightName, double yaw, double yawRate, 
         double pitch, double pitchRate, 
         double roll, double rollRate) {
         SetRobotOrientation_INTERNAL(limelightName, yaw, yawRate, pitch, pitchRate, roll, rollRate, false);
     }
 
     private static void SetRobotOrientation_INTERNAL(String limelightName, double yaw, double yawRate, 
         double pitch, double pitchRate, 
         double roll, double rollRate, boolean flush) {
 
         double[] entries = new double[6];
         entries[0] = yaw;
         entries[1] = yawRate;
         entries[2] = pitch;
         entries[3] = pitchRate;
         entries[4] = roll;
         entries[5] = rollRate;
         getLimelightNTTableEntry(limelightName, "robot_orientation_set").setDoubleArray(entries);
         if(flush)
         {
             Flush();
         }
     }
 
     private static PoseEstimate getBotPoseEstimate(String limelightName, String entryName, boolean isMegaTag2) {
         DoubleArrayEntry poseEntry = getLimelightDoubleArrayEntry(limelightName, entryName);
         
         TimestampedDoubleArray tsValue = poseEntry.getAtomic();
         double[] poseArray = tsValue.value;
         long timestamp = tsValue.timestamp;
         
         if (poseArray.length == 0) {
             // Handle the case where no data is available
             return null; // or some default PoseEstimate
         }
     
         var pose = toPose2D(poseArray);
         double latency = extractArrayEntry(poseArray, 6);
         int tagCount = (int)extractArrayEntry(poseArray, 7);
         double tagSpan = extractArrayEntry(poseArray, 8);
         double tagDist = extractArrayEntry(poseArray, 9);
         double tagArea = extractArrayEntry(poseArray, 10);
         
         // Convert server timestamp from microseconds to seconds and adjust for latency
         double adjustedTimestamp = (timestamp / 1000000.0) - (latency / 1000.0);
     
         RawFiducial[] rawFiducials = new RawFiducial[tagCount];
         int valsPerFiducial = 7;
         int expectedTotalVals = 11 + valsPerFiducial * tagCount;
     
         if (poseArray.length != expectedTotalVals) {
             // Don't populate fiducials
         } else {
             for(int i = 0; i < tagCount; i++) {
                 int baseIndex = 11 + (i * valsPerFiducial);
                 int id = (int)poseArray[baseIndex];
                 double txnc = poseArray[baseIndex + 1];
                 double tync = poseArray[baseIndex + 2];
                 double ta = poseArray[baseIndex + 3];
                 double distToCamera = poseArray[baseIndex + 4];
                 double distToRobot = poseArray[baseIndex + 5];
                 double ambiguity = poseArray[baseIndex + 6];
                 rawFiducials[i] = new RawFiducial(id, txnc, tync, ta, distToCamera, distToRobot, ambiguity);
             }
         }
     
         return new PoseEstimate(pose, adjustedTimestamp, latency, tagCount, tagSpan, tagDist, tagArea, rawFiducials, isMegaTag2);
     }

}
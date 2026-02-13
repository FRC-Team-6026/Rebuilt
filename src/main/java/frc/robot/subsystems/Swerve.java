package frc.robot.subsystems;

import com.kauailabs.navx.frc.AHRS;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.util.PathPlannerLogging;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructArrayPublisher;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.lib.configs.Kraken.SwerveModuleInfo;
import frc.robot.Constants;

public class Swerve extends SubsystemBase {
  private final AHRS gyro;

  private SwerveDriveOdometry swerveOdometry;
  private SwerveModule[] mSwerveMods;

  private static boolean negativePitch = false;

  public SysIdRoutine sysIdRoutine;
  
  private Field2d field = new Field2d();
  
  StructArrayPublisher<SwerveModuleState> pub = NetworkTableInstance.getDefault()
  .getStructArrayTopic("MyStates", SwerveModuleState.struct).publish();

  public Swerve() {
    gyro = new AHRS();
    gyro.reset();
    zeroGyro();
    // gyro.setAngleAdjustment(Constants.Setup.gyroAngleOffset);

    mSwerveMods = new SwerveModule[4];

    for(int i = 0; i <= 3; i++){
        mSwerveMods[i] = new SwerveModule(new SwerveModuleInfo(i));
    }
    
    swerveOdometry = new SwerveDriveOdometry(Constants.Swerve.swerveKinematics, getAngle(), getPositions());

    // Load the RobotConfig from the GUI settings. You should probably
    // store this in your Constants file
    RobotConfig config = null;
    try{
      config = RobotConfig.fromGUISettings();
    } catch (Exception e) {
      // Handle exception as needed
      e.printStackTrace();
    }

    AutoBuilder.configure(
      this::getPose, 
      this::resetOdometry, 
      this::getSpeeds,
      this::driveRobotRelative,
      Constants.Swerve.pathFollowerConfig,
      config,
      () -> {
          // Boolean supplier that controls when the path will be mirrored for the red alliance
          // This will flip the path being followed to the red side of the field.
          // THE ORIGIN WILL REMAIN ON THE BLUE SIDE

          var alliance = DriverStation.getAlliance();
          if (alliance.isPresent()) {
              return alliance.get() == DriverStation.Alliance.Red;
          }
          return false;
      },
      this
    );

    // Set up custom logging to add the current path to a field 2d widget

    //Check for later after current competitions
    PathPlannerLogging.setLogActivePathCallback((poses) -> field.getObject("path").setPoses(poses));

    SmartDashboard.putData("Field", field);

    SmartDashboard.putData("Swerve Drive", new Sendable() {
      @Override
      public void initSendable(SendableBuilder builder) {
        builder.setSmartDashboardType("SwerveDrive");

        builder.addDoubleProperty("Front Left Angle", () -> mSwerveMods[0].getState().angle.getRadians(), null);
        builder.addDoubleProperty("Front Left Velocity", () -> mSwerveMods[0].getState().speedMetersPerSecond, null);

        builder.addDoubleProperty("Front Right Angle", () -> mSwerveMods[1].getState().angle.getRadians(), null);
        builder.addDoubleProperty("Front Right Velocity", () -> mSwerveMods[1].getState().speedMetersPerSecond, null);

        builder.addDoubleProperty("Back Left Angle", () -> mSwerveMods[2].getState().angle.getRadians(), null);
        builder.addDoubleProperty("Back Left Velocity", () -> mSwerveMods[2].getState().speedMetersPerSecond, null);

        builder.addDoubleProperty("Back Right Angle", () -> mSwerveMods[3].getState().angle.getRadians(), null);
        builder.addDoubleProperty("Back Right Velocity", () -> mSwerveMods[3].getState().speedMetersPerSecond, null);

        builder.addDoubleProperty("Robot Angle", () -> getAngle().getRadians(), null);
      }
    });
    

    // SysId - the actual SysId routine. Configures settings and creates the callable function
    sysIdRoutine = new SysIdRoutine(
      new SysIdRoutine.Config(
        null,
        Units.Volts.of(5),
        Units.Seconds.of(6.0)
      ),
      new SysIdRoutine.Mechanism(
        (voltage) -> this.runVolts(voltage),
        null, // No log consumer, since data is recorded by URCL
        this
      )
    );
  }


  @Override
  public void periodic() {
    swerveOdometry.update(getAngle(), getPositions());
    report();
    
    pub.set(getStates());
    SmartDashboard.putString("Pose (x, y, rot)", "("+
      Math.round(getPose().getX()*100)/100.0+", "+
      Math.round(getPose().getY()*100)/100.0+", "+
      Math.round(getPose().getRotation().getDegrees())+
      ")");
  }

  public void drive(
      Translation2d translation, double rotation, boolean fieldRelative, boolean isOpenLoop) {
    SwerveModuleState[] swerveModuleStates =
        Constants.Swerve.swerveKinematics.toSwerveModuleStates(
            fieldRelative
                ? ChassisSpeeds.fromFieldRelativeSpeeds(
                    translation.getX(), translation.getY(), rotation, getAngle())
                : new ChassisSpeeds(translation.getX(), translation.getY(), rotation));
    SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleStates, Constants.Swerve.maxSpeed);
    for (SwerveModule mod : mSwerveMods) {
      mod.setDesiredState(swerveModuleStates[mod.moduleNumber], isOpenLoop);
      var modState = swerveModuleStates[mod.moduleNumber];
      
      SmartDashboard.putNumber("Mod " + mod.moduleNumber + " desired angle: ", modState.angle.getDegrees());
      SmartDashboard.putNumber("Mod " + mod.moduleNumber + " desired velocity: ", modState.speedMetersPerSecond);
    }
  }

  /* Used by SwerveControllerCommand in Auto */
  public void setModuleStates(SwerveModuleState[] desiredStates) {
    SwerveDriveKinematics.desaturateWheelSpeeds(desiredStates, Constants.Swerve.maxSpeed);

    for (SwerveModule mod : mSwerveMods) {
      mod.setDesiredState(desiredStates[mod.moduleNumber], false);
    }
  }

  public void driveRobotRelative(ChassisSpeeds robotRelativeSpeeds) {
    ChassisSpeeds targetSpeeds = ChassisSpeeds.discretize(robotRelativeSpeeds, 0.02);

    SwerveModuleState[] targetStates = Constants.Swerve.swerveKinematics.toSwerveModuleStates(targetSpeeds);
    setModuleStates(targetStates);
  }

  public Pose2d getPose() {
    return swerveOdometry.getPoseMeters();
  }

  public void resetOdometry(Pose2d pose) {
    swerveOdometry.resetPosition(getAngle(), getPositions(), pose);
  }

  public SwerveModuleState[] getStates() {
    SwerveModuleState[] states = new SwerveModuleState[4];
    for (SwerveModule mod : mSwerveMods) {
      states[mod.moduleNumber] = mod.getState();
    }
    return states;
  }

  public SwerveModulePosition[] getPositions() {
    SwerveModulePosition[] positions = new SwerveModulePosition[4];
    for (SwerveModule mod : mSwerveMods) {
      positions[mod.moduleNumber] = mod.getPostion();
    }
    return positions;
  }

  public ChassisSpeeds getSpeeds() {
    return Constants.Swerve.swerveKinematics.toChassisSpeeds(getStates());
  }

  public void zeroGyro() {
    gyro.zeroYaw();
    gyro.setAngleAdjustment(Constants.Setup.gyroAngleOffset);
    negativePitch = false;
  }

  public Rotation2d getAngle() {
    return (Constants.Swerve.invertGyro)
        ? Rotation2d.fromDegrees(360 - gyro.getAngle())
        : Rotation2d.fromDegrees(gyro.getAngle());
  }

  public void resetToAbsolute() {
    for (SwerveModule mod : mSwerveMods) {
        mod.resetToAbsolute();
    }
  }

  public float getPitch() {
    if (negativePitch){
      return -gyro.getPitch();
    } else {
      return gyro.getPitch();
    }
  }

  public void invertGyro() {
    gyro.setAngleAdjustment(180);
    negativePitch = true;
  }

  public AHRS getGyro() {
    return gyro;
  }

  public void report() {
    for (SwerveModule mod : mSwerveMods) {
      SmartDashboard.putNumber(
          "Mod " + mod.moduleNumber + " Cancoder", mod.getCanCoder().getDegrees());
      SmartDashboard.putNumber(
          "Mod " + mod.moduleNumber + " Integrated", mod.getState().angle.getDegrees());
      SmartDashboard.putNumber(
          "Mod " + mod.moduleNumber + " Velocity", mod.getState().speedMetersPerSecond);      
    }
  }

  // SysId - function for setting voltage to motor.
  // This function just passes voltage value to each module.
  public void runVolts(Voltage voltage) {
    for (SwerveModule mod : mSwerveMods) {
      mod.setVoltage(voltage);
    }
  }

  // SysId - Method that sets wheels to forward.
  public Command testInit() {
    // Check to make sure our swerve.setAngle() jitter protection doesnt cause problems with this
    return new InstantCommand(
      () -> setModuleStates(new SwerveModuleState[] {
        new SwerveModuleState(0.0, new Rotation2d(Constants.Setup.angleOffsets[0])),
        new SwerveModuleState(0.0, new Rotation2d(Constants.Setup.angleOffsets[1])),
        new SwerveModuleState(0.0, new Rotation2d(Constants.Setup.angleOffsets[2])),
        new SwerveModuleState(0.0, new Rotation2d(Constants.Setup.angleOffsets[3]))
      })
    );
  }

  // SysID - 4 commands for the 4 SysID tests. Each one can be bound to a button, and cancelled when the button is released.
  public Command SysIDQuasiF() { return sysIdRoutine.quasistatic(SysIdRoutine.Direction.kForward); }
  public Command SysIDQuasiR() { return sysIdRoutine.quasistatic(SysIdRoutine.Direction.kReverse); }
  public Command SysIDDynF() { return sysIdRoutine.dynamic(SysIdRoutine.Direction.kForward); }
  public Command SysIDDynR() { return sysIdRoutine.dynamic(SysIdRoutine.Direction.kReverse); }
}
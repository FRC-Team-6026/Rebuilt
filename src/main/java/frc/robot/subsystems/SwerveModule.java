package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.CANcoder;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkClosedLoopController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.units.measure.Voltage;
import frc.robot.Constants;
import frc.lib.Items.SparkMax.SparkController;
import frc.lib.configs.Sparkmax.SwerveModuleInfo;
import frc.lib.math.OnboardModuleState;

public class SwerveModule {
  public int moduleNumber;
  private Rotation2d lastAngle;
  private Rotation2d angleOffset;

  private SparkController drive;
  private SparkController angle;

  private SparkMax angleMotor;
  private SparkMax driveMotor;

  private RelativeEncoder driveEncoder;
  private RelativeEncoder integratedAngleEncoder;
  private CANcoder angleEncoder;

  private final SparkClosedLoopController driveController;
  private final SparkClosedLoopController angleController;

  private final SimpleMotorFeedforward feedforward =
      new SimpleMotorFeedforward(
          Constants.SVA.driveMotorsSVA[0], Constants.SVA.driveMotorsSVA[1], Constants.SVA.driveMotorsSVA[2]);

  public SwerveModule(SwerveModuleInfo Info) {
    this.moduleNumber = Info.moduleNumber;
    this.angleOffset = Rotation2d.fromDegrees(Info.angleOffset);

    this.drive = Info.drive;
    this.angle = Info.angle;

    /* Angle Encoder Config */
    angleEncoder = Info.cancoder;

    /* Angle Motor Config */
    angleMotor = angle.spark;
    integratedAngleEncoder = angleMotor.getEncoder();
    angleController = angleMotor.getClosedLoopController();

    /* Drive Motor Config */
    driveMotor = drive.spark;
    driveEncoder = driveMotor.getEncoder();
    driveController = driveMotor.getClosedLoopController();

    lastAngle = getState().angle;
  }

  public void setDesiredState(SwerveModuleState desiredState, boolean isOpenLoop) {
    
    // Custom optimize command, since default WPILib optimize assumes continuous controller which
    // REV and CTRE are not

    desiredState = OnboardModuleState.optimize(desiredState, getState().angle);

      setAngle(desiredState);
      setSpeed(desiredState, isOpenLoop);
  }

  void resetToAbsolute() {
    double absolutePosition = getCanCoder().getDegrees() - angleOffset.getDegrees();
    integratedAngleEncoder.setPosition(absolutePosition);
  }

  private void setSpeed(SwerveModuleState desiredState, boolean isOpenLoop) {
    if (isOpenLoop) {
      double percentOutput = desiredState.speedMetersPerSecond / Constants.Swerve.maxSpeed;
      driveMotor.set(percentOutput);
    } else {
      driveController.setReference(
          desiredState.speedMetersPerSecond,
          SparkMax.ControlType.kVelocity,
          ClosedLoopSlot.kSlot0,
          feedforward.calculate(desiredState.speedMetersPerSecond));
    }
  }

  // SysId - directly sets voltage value to motor
  public void setVoltage(Voltage voltage) {
    driveController.setReference(voltage.magnitude(), ControlType.kVoltage);
  }

  private void setAngle(SwerveModuleState desiredState) {
    // Prevent rotating module if speed is less then 1%. Prevents jittering.
    Rotation2d angle =
        (Math.abs(desiredState.speedMetersPerSecond) <= (Constants.Swerve.maxSpeed * 0.01))? 
        lastAngle : desiredState.angle;

    angleController.setReference(angle.getDegrees(), SparkMax.ControlType.kPosition);
    lastAngle = angle;
  }

  private Rotation2d getAngle() {
    return Rotation2d.fromDegrees(integratedAngleEncoder.getPosition());
  }

  public Rotation2d getCanCoder() {
    return Rotation2d.fromRotations(angleEncoder.getAbsolutePosition().getValueAsDouble());
  }

  public SwerveModuleState getState() {
    return new SwerveModuleState(driveEncoder.getVelocity(), getAngle());
  }

  public SwerveModulePosition getPostion() {
    return new SwerveModulePosition(driveEncoder.getPosition(), getAngle());
  }
}
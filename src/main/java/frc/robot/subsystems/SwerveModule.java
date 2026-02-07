package frc.robot.subsystems;

import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
// import com.revrobotics.spark.SparkMax;
// import com.revrobotics.RelativeEncoder;
// import com.revrobotics.spark.ClosedLoopSlot;
// import com.revrobotics.spark.SparkBase.ControlType;
// import com.revrobotics.spark.SparkClosedLoopController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.units.measure.Voltage;
import frc.robot.Constants;
// import frc.lib.Items.SparkMax.SparkController;
import frc.lib.configs.Kraken.SwerveModuleInfo;
import frc.lib.math.OnboardModuleState;


// Do we want to change over to this swerve code, or work with our own? Probably stick with our own
// https://v6.docs.ctr-electronics.com/en/latest/docs/api-reference/mechanisms/swerve/swerve-overview.html

public class SwerveModule {
  public int moduleNumber;
  private Rotation2d lastAngle;
  private Rotation2d angleOffset;

  // private SparkController drive;
  // private SparkController angle;

  // private SparkMax angleMotor;
  // private SparkMax driveMotor;

  private TalonFX angleMotor_talon; // rename to angleMotor once we fully recode?
  private TalonFX driveMotor_talon; // rename to driveMotor once we fully recode?
  private VelocityVoltage velocityControl;
  private VoltageOut voltageControl;
  private PositionVoltage positionControl;

  // private RelativeEncoder driveEncoder;
  // private RelativeEncoder integratedAngleEncoder;
  private CANcoder angleEncoder;

  // private final SparkClosedLoopController driveController;
  // private final SparkClosedLoopController angleController;

  private final SimpleMotorFeedforward feedforward =
      new SimpleMotorFeedforward(
          Constants.SVA.driveMotorsSVA[0], Constants.SVA.driveMotorsSVA[1], Constants.SVA.driveMotorsSVA[2]);

  public SwerveModule(SwerveModuleInfo Info) {
    this.moduleNumber = Info.moduleNumber;
    this.angleOffset = Rotation2d.fromDegrees(Info.angleOffset);

    angleMotor_talon = Info.angle.motor;
    driveMotor_talon = Info.drive.motor;

    // this.drive = Info.drive;
    // this.angle = Info.angle;

    /* Angle Encoder Config */
    angleEncoder = Info.cancoder;

    /* Angle Motor Config */
    // angleMotor = angle.spark;
    // integratedAngleEncoder = angleMotor.getEncoder();
    // angleController = angleMotor.getClosedLoopController();

    /* Drive Motor Config */
    // driveMotor = drive.spark;
    // driveEncoder = driveMotor.getEncoder();
    // driveController = driveMotor.getClosedLoopController();

    velocityControl = new VelocityVoltage(0.0);
    voltageControl = new VoltageOut(0.0);
    positionControl = new PositionVoltage(0.0);

    lastAngle = getState().angle;
  }

  /** Sets the speed for the main drive motor to achieve, and the angle for the angle motor to point the wheel at
   * 
   * @param desiredState an object with the desired direction and speed for the wheel to point to and drive at
   * @param isOpenLoop currently not implemented. runs closed loop always, getting feedback from the motor system and trying to hit the correct speed
   */
  public void setDesiredState(SwerveModuleState desiredState, boolean isOpenLoop) {
    
    // Custom optimize command, since default WPILib optimize assumes continuous controller which
    // REV and CTRE are not

    // TODO - According to this page, our Kraken's should be able to do this? I think it requires special configuration of the cancoders, however. Not sure.
    // See Continuous Mechanism Wrap towards bottom of page
    // https://v6.docs.ctr-electronics.com/en/latest/docs/api-reference/device-specific/talonfx/closed-loop-requests.html

    desiredState = OnboardModuleState.optimize(desiredState, getState().angle);

    setAngle(desiredState);
    setSpeed(desiredState, isOpenLoop);
  }

  void resetToAbsolute() {
    double absolutePosition = getCanCoder().getDegrees() - angleOffset.getDegrees();

    // integratedAngleEncoder.setPosition(absolutePosition);
    angleMotor_talon.setPosition(absolutePosition/360.0);

    lastAngle = Rotation2d.fromDegrees(absolutePosition);;
  }

  /** Sets the speed of the main drive motor only. Usually called from the setDesiredState method
   * 
   * @param desiredState an object with the desired speed for the main drive motor to achieve
   * @param isOpenLoop currently not implemented. runs closed loop, getting feedback from the motor system and trying to hit the correct speed
   */
  private void setSpeed(SwerveModuleState desiredState, boolean isOpenLoop) {
    if (isOpenLoop) {
      // double percentOutput = desiredState.speedMetersPerSecond / Constants.Swerve.maxSpeed;
      // driveMotor.set(percentOutput);

      driveMotor_talon.setControl(velocityControl
        .withVelocity(desiredState.speedMetersPerSecond)
        .withFeedForward(feedforward.calculate(desiredState.speedMetersPerSecond))
      );
    } else {
      // driveController.setReference(
      //     desiredState.speedMetersPerSecond,
      //     SparkMax.ControlType.kVelocity,
      //     ClosedLoopSlot.kSlot0,
      //     feedforward.calculate(desiredState.speedMetersPerSecond));

      driveMotor_talon.setControl(velocityControl
        .withVelocity(desiredState.speedMetersPerSecond)
        .withFeedForward(feedforward.calculate(desiredState.speedMetersPerSecond))
      );
    }
  }

  // SysId - directly sets voltage value to motor
  public void setVoltage(Voltage voltage) {
    // driveController.setReference(voltage.magnitude(), ControlType.kVoltage);
    driveMotor_talon.setControl(voltageControl.withOutput(voltage));
  }

  private void setAngle(SwerveModuleState desiredState) {
    // Prevent rotating module if speed is less then 1%. Prevents jittering.
    Rotation2d angle =
        (Math.abs(desiredState.speedMetersPerSecond) <= (Constants.Swerve.maxSpeed * 0.01))? 
        lastAngle : desiredState.angle;

    // angleController.setReference(angle.getDegrees(), SparkMax.ControlType.kPosition);
    angleMotor_talon.setControl(positionControl.withPosition(angle.getRotations()));

    lastAngle = angle;
  }

  private Rotation2d getAngle() {
    // return Rotation2d.fromDegrees(integratedAngleEncoder.getPosition());
    return Rotation2d.fromRotations(angleMotor_talon.getPosition().getValueAsDouble());
  }

  public Rotation2d getCanCoder() {
    return Rotation2d.fromRotations(angleEncoder.getAbsolutePosition().getValueAsDouble());
  }

  public SwerveModuleState getState() {
    // return new SwerveModuleState(driveEncoder.getVelocity(), getAngle());
    return new SwerveModuleState(driveMotor_talon.getVelocity().getValueAsDouble(), getAngle());
  }

  public SwerveModulePosition getPostion() {
    // return new SwerveModulePosition(driveEncoder.getPosition(), getAngle());
    return new SwerveModulePosition(driveMotor_talon.getPosition().getValueAsDouble(), getAngle());
  }
}
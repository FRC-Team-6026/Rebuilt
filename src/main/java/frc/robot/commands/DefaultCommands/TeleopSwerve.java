package frc.robot.commands.DefaultCommands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.Preferences;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.Limelight;
import frc.robot.subsystems.Swerve;
import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

public class TeleopSwerve extends Command {
  private Swerve s_Swerve;
  private Limelight s_Limelight;

  private DoubleSupplier translationSup;
  private DoubleSupplier strafeSup;
  private DoubleSupplier rotationSup;

  private BooleanSupplier autoaim;
  private BooleanSupplier robotCentricSup;

  private SlewRateLimiter translationLimiter = new SlewRateLimiter(2.0);
  private SlewRateLimiter strafeLimiter = new SlewRateLimiter(2.0);
  private SlewRateLimiter rotationLimiter = new SlewRateLimiter(2.0);

  public TeleopSwerve(
      Swerve s_Swerve,
      Limelight s_Limelight,
      DoubleSupplier translationSup,
      DoubleSupplier strafeSup,
      DoubleSupplier rotationSup,
      BooleanSupplier autoaim,
      BooleanSupplier robotCentricSup) {
    this.s_Swerve = s_Swerve;
    addRequirements(s_Swerve);
    this.s_Limelight = s_Limelight;
    addRequirements(s_Limelight);

    this.translationSup = translationSup;
    this.strafeSup = strafeSup;
    this.rotationSup = rotationSup;
    this.autoaim = autoaim;
    this.robotCentricSup = robotCentricSup;
  }

  @Override
  public void initialize()
  {
  }

  @Override
  public void execute() {
    double translationVal;
    double strafeVal;
    double rotationVal;

    /* Get Values, Deadband*/
    translationVal = MathUtil.applyDeadband(translationSup.getAsDouble(), Constants.Swerve.stickDeadband);

    //cubes input(s) to give finer control at low end
    translationVal = translationVal * translationVal * translationVal;

    // joystick inputs need deadband/cube/slewrate calc, but limelight inputs should only need slewrate

    if (autoaim.getAsBoolean()) {
      strafeVal = s_Limelight.getTX(
        Math.atan(
          strafeSup.getAsDouble()/s_Limelight.getTZ()
        )
      )*Preferences.getDouble("Aim Strafe Power", 1.0)/100.0;
      rotationVal = -s_Limelight.getTX(
        Math.atan(
          strafeSup.getAsDouble()/s_Limelight.getTZ()
        )
      )*Preferences.getDouble("Aim Rotation Power", 1.0)/100.0;
    } else {
      strafeVal = MathUtil.applyDeadband(strafeSup.getAsDouble(), Constants.Swerve.stickDeadband);
      strafeVal = strafeVal * strafeVal * strafeVal;
      rotationVal = MathUtil.applyDeadband(rotationSup.getAsDouble(), Constants.Swerve.stickDeadband);
      rotationVal = rotationVal * rotationVal * rotationVal;
    }

    //limit change per input to avoid slamming the motors
    translationVal = translationLimiter.calculate(translationVal);
    strafeVal = strafeLimiter.calculate(strafeVal);
    rotationVal = rotationLimiter.calculate(rotationVal);

    /* Drive */
    s_Swerve.drive(
      new Translation2d(translationVal, strafeVal).times(Constants.Swerve.maxSpeed),
      rotationVal * Constants.Swerve.maxAngularVelocity,
      !robotCentricSup.getAsBoolean(),
      false
    );
  }
}
package frc.robot.commands;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.Preferences;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.Limelight;
import frc.robot.subsystems.Swerve;

// Autoaim command specifically for autonomous
public class AutoAim extends Command {
    private Swerve s_Swerve;
    private Limelight s_Limelight;
    public AutoAim (
        Swerve swerve,
        Limelight limelight
    ) {
        s_Swerve = swerve;
        s_Limelight = limelight;
    }

    public void execute() {
        double rotationVal = -s_Limelight.getTX()
                * Preferences.getDouble("Aim Rotation Power", 1.0) / 100.0;
                
        s_Swerve.drive(
            new Translation2d(0, 0),
            rotationVal * Constants.Swerve.maxAngularVelocity,
            false,
            false
        );
    }

    @Override
    public void end(boolean interrupted) {
        s_Swerve.drive(
            new Translation2d(0, 0),
            0,
            false,
            false);
    }

    @Override
    public boolean isFinished() {
        return Math.abs(s_Limelight.getTX()) < 2;
    }
}

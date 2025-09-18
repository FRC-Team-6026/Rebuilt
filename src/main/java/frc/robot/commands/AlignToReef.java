package frc.robot.commands;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Swerve;
import frc.robot.subsystems.Limelight;

public class AlignToReef extends Command{
    public Swerve swerve;
    public Limelight limelight;

    public AlignToReef(Swerve swerve, Limelight limelight) {
        this.swerve = swerve;
        this.limelight = limelight;
        
        addRequirements(swerve);
    }

    @Override
    public void execute() {
        swerve.drive(
            new Translation2d(limelight.getTZ(), limelight.getTX()),
            0,
            false,
            false
        );
    }

    @Override
    public void end(boolean interrupted) {
        if (!interrupted) {
            swerve.drive(
                new Translation2d(0, 0),
                0,
                false,
                false
            );
        }
    }
}

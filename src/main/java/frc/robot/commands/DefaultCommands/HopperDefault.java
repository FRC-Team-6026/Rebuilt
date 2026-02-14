package frc.robot.commands.DefaultCommands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.Preferences;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Hopper;

// Useful for testing the hopper. Ideally, we don't even use this
// command, but only set the hopper via the commands in the class.

public class HopperDefault extends Command{
    private Hopper s_Hopper;
    private DoubleSupplier speedSup;

    public HopperDefault(
        Hopper s_Hopper,
        DoubleSupplier speedSup
    ) {
        this.s_Hopper = s_Hopper;
        addRequirements(s_Hopper);
        
        this.speedSup = speedSup;
    }

    @Override
    public void execute() {
        double speedPref = Preferences.getDouble("Hopper Volts", 1);

        // Applying deadband so thumbsticks that are slightly off dont trigger command
        double speed = MathUtil.applyDeadband(speedSup.getAsDouble(), 0.1);
        
        s_Hopper.setVoltage(speed * speedPref, true);
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
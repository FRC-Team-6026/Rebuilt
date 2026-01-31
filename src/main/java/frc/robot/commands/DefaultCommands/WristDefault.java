package frc.robot.commands.DefaultCommands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.Preferences;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Elevator_old;
import frc.robot.subsystems.Wrist;

// Left from previous year as example, not intended for 2026

// Useful for testing the wrist. Ideally, we don't even use this
// command, but only set the wrist via the SetWristCommand.

public class WristDefault extends Command{
    private Wrist s_Wrist;
    private DoubleSupplier speedSup;
    public Elevator_old s_Elevator;
    private boolean voltControl;

    private Timer wristTimer;

    public WristDefault(
        Wrist s_Wrist,
        Elevator_old s_Elevator,
        DoubleSupplier speedSup
    ) {
        this.s_Wrist = s_Wrist;
        this.s_Elevator = s_Elevator;
        addRequirements(s_Wrist);

        wristTimer = new Timer();
        
        this.speedSup = speedSup;
    }

    @Override
    public void initialize(){
        wristTimer.reset();
        wristTimer.start();

        voltControl = false;
    }

    @Override
    public void execute(){
        double speedPref = Preferences.getDouble("WristSpeed", 0.2);

        // Applying deadband so thumbsticks that are slightly off dont trigger command
        double speed = MathUtil.applyDeadband(speedSup.getAsDouble(), 0.1);
        
        if (speed == 0.0 && voltControl) {
            s_Wrist.setAngle(s_Wrist.getAngle());
            voltControl = false;
        }
        else if (speed != 0.0) {
            s_Wrist.setVoltage(speed * speedPref);
            voltControl = true;
        }
        
        wristTimer.reset();
    }
    
    @Override
    public void end(boolean interrupted) {
        s_Wrist.setAngle(s_Wrist.getAngle());
    }

    @Override
    public boolean isFinished(){
        return false;
    }
}
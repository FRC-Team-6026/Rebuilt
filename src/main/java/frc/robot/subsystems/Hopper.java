package frc.robot.subsystems;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkBase.ControlType;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj.Preferences;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.Items.SparkMax.SparkController;
import frc.lib.configs.Sparkmax.SparkControllerInfo;
import frc.robot.Constants;

public class Hopper extends SubsystemBase {
    private SparkController hopperSpark;
    private RelativeEncoder hopperEncoder;
    private SparkClosedLoopController hopperController;

    public Hopper() {
        this.hopperSpark = new SparkController(Constants.Setup.hopperSpark, new SparkControllerInfo().hopper(),
            Constants.Hopper.minPercent, Constants.Hopper.maxPercent,
            Constants.Hopper.maxAngle, Constants.Hopper.minAngle);

        this.hopperEncoder = hopperSpark.sparkEncode;
        this.hopperController = hopperSpark.sparkControl;
    }

    public Command deploy() {
        // double target = Preferences.getDouble("Hopper Deploy Target", 50.0);
        return new RunCommand(() -> hopperController.setReference(Preferences.getDouble("Hopper Deploy Target", 50.0), ControlType.kPosition, ClosedLoopSlot.kSlot0, getFF()))
        .until(() -> Math.abs(hopperEncoder.getPosition()-Preferences.getDouble("Hopper Deploy Target", 50.0)) < Constants.Hopper.tolerance);
    }

    public Command retract() {
        return new RunCommand(() -> hopperController.setReference(0, ControlType.kPosition, ClosedLoopSlot.kSlot0, getFF()))
        .until(() -> Math.abs(hopperEncoder.getPosition()) < Constants.Hopper.tolerance);
    }

    public void setVoltage(double volts) {
        setVoltage(volts, false);
    }

    public void setVoltage(double volts, boolean FFenable) {
        volts = MathUtil.clamp(volts, -Constants.Hopper.maxVoltage, Constants.Hopper.maxVoltage);
        if (FFenable)
            hopperController.setReference(volts, ControlType.kVoltage, ClosedLoopSlot.kSlot0, getFF());
        else
            hopperController.setReference(volts, ControlType.kVoltage);
    }

    public class HomingCommand extends Command {
        public double current; // Amps from SparkMax; Spikes when colliding with something.
        public double firstCurrent;
        public Alert c_c;
        public Alert c_fc;
        public Alert c_triggered;

        public double lastPos;
        public boolean firstCycle = true;
        public void initialize() {
            current = 0;
            firstCurrent = 0;
            c_c = new Alert("Current: X", AlertType.kInfo);
            c_fc = new Alert("First Current: X", AlertType.kInfo);
            c_triggered = new Alert("Limit Triggered", AlertType.kWarning);

            lastPos = 0;
            hopperController.setReference(0, ControlType.kPosition, ClosedLoopSlot.kSlot0, -0.12);
        }
        public void execute() {
            if (lastPos != 0) firstCycle = false;
            lastPos = hopperEncoder.getPosition();
            hopperEncoder.setPosition(5);

            if (current != 0) { firstCycle = false; firstCurrent = current; }
            current = hopperSpark.spark.getOutputCurrent();
            hopperEncoder.setPosition(5);

            c_fc.setText("First Current: " + firstCurrent);
            c_fc.set(true);

            // XXX - Test different current limits
            if ((current >= firstCurrent*Preferences.getDouble("Hopper Trigger (Amps)", 1.5)) && !firstCycle) {
                c_triggered.set(true);
                c_c.setText("Current: " + current);
                c_c.set(true);
            }
        }
        public void end(boolean interrupted)    { hopperEncoder.setPosition(-6); }
        public boolean isFinished() { return (lastPos >= 5) && !firstCycle; }
    }

    public Command homeCommand() {
        Command result = new HomingCommand();
        result.addRequirements(this);
        return result;
    }

    public double getFF() { return -Math.sin( (hopperEncoder.getPosition()-20.0) *Math.PI/360)*0.2; }
}

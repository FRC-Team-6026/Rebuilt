package frc.robot.subsystems;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkBase.ControlType;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
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
    private static double tiltDegrees = 18;

    public Hopper() {
        this.hopperSpark = new SparkController(Constants.Setup.hopperSpark, new SparkControllerInfo().hopper(),
            Constants.Hopper.minPercent, Constants.Hopper.maxPercent,
            Constants.Hopper.maxAngle, Constants.Hopper.minAngle);

        this.hopperEncoder = hopperSpark.sparkEncode;
        this.hopperController = hopperSpark.sparkControl;
    }

    public void periodic() {
        SmartDashboard.putNumber("Hopper Position", hopperEncoder.getPosition());
    }

    // TODO - change hopper deploy target by -5 deg after hopper redesign bumps the angle out by 5
    public Command deploy() {
        return new RunCommand(() -> hopperController.setSetpoint(Preferences.getDouble("Hopper Deploy Target", 100.0), ControlType.kPosition, ClosedLoopSlot.kSlot0, getFF()))
        .until(() -> Math.abs(hopperEncoder.getPosition()-Preferences.getDouble("Hopper Deploy Target", 100.0)) < Constants.Hopper.tolerance);
    }

    public Command tilt() {
        return new RunCommand(() -> hopperController.setSetpoint(Preferences.getDouble("Hopper Deploy Target", 100.0)-tiltDegrees, ControlType.kPosition, ClosedLoopSlot.kSlot0, getFF()))
        .until(() -> Math.abs(hopperEncoder.getPosition()-Preferences.getDouble("Hopper Deploy Target", 100.0)-tiltDegrees) < Constants.Hopper.tolerance);
    }

    public Command retract() {
        return new RunCommand(() -> hopperController.setSetpoint(0, ControlType.kPosition, ClosedLoopSlot.kSlot0, getFF()))
        .until(() -> Math.abs(hopperEncoder.getPosition()) < Constants.Hopper.tolerance);
    }

    public double getPosition() {
        return hopperEncoder.getPosition();
    }

    public void setVoltage(double volts) {
        setVoltage(volts, false);
    }

    public void setVoltage(double volts, boolean FFenable) {
        volts = MathUtil.clamp(volts, -Constants.Hopper.maxVoltage, Constants.Hopper.maxVoltage);
        if (FFenable)
            hopperController.setSetpoint(volts, ControlType.kVoltage, ClosedLoopSlot.kSlot0, getFF());
        else
            hopperController.setSetpoint(volts, ControlType.kVoltage);
    }

    public class HomingCommand extends Command {
        public double current; // Amps from SparkMax; Spikes when colliding with something.
        public Alert c_c;
        public Alert c_fc;
        public Alert c_triggered;

        public double lastPos;
        public int cycles = 0;
        public void initialize() {
            current = 0;
            c_c = new Alert("Current: X", AlertType.kInfo);
            c_fc = new Alert("First Current: X", AlertType.kInfo);
            c_triggered = new Alert("Limit Triggered", AlertType.kWarning);

            lastPos = 0;
            hopperEncoder.setPosition(4.5);
            hopperController.setSetpoint(0, ControlType.kPosition, ClosedLoopSlot.kSlot0, -0.12);
        }
        public void execute() {
            cycles++;
            lastPos = hopperEncoder.getPosition();
            hopperEncoder.setPosition(5);

            // IN PROGRESS - Test different current limits
            // if (current != 0) { firstCycle = false; }
            // current = hopperSpark.spark.getOutputCurrent();

            // c_fc.setText("Current: " + current);
            // c_fc.set(true);

            // if ((current >= Preferences.getDouble("Hopper Trigger (Amps)", 0.5)) && !firstCycle) {
            //     c_triggered.set(true);
            //     c_c.setText("Final Current: " + current);
            //     c_c.set(true);
            // }
        }
        public void end(boolean interrupted)    { hopperEncoder.setPosition(-5); }
        public boolean isFinished() { return (lastPos >= 5) && (cycles > 10); }
    }

    public Command homeCommand() {
        Command result = new HomingCommand();
        result.addRequirements(this);
        return result;
    }

    public double getFF() { return -Math.sin( (hopperEncoder.getPosition()-15.0) *Math.PI/360)*0.2; }
}

package frc.robot.subsystems;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkBase.ControlType;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.Preferences;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.Items.SparkMax.SparkController;
import frc.lib.configs.Sparkmax.SparkControllerInfo;
import frc.robot.Constants;

// TODO - possibly figure out a homing mechanism?

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
        public double lastPos;
        public boolean firstCycle = true;
        public void initialize() {
            lastPos = 0;
            hopperController.setReference(0, ControlType.kPosition, ClosedLoopSlot.kSlot0, -0.12);
        }
        public void execute() {
            if (lastPos != 0) firstCycle = false;
            lastPos = hopperEncoder.getPosition();
            hopperEncoder.setPosition(5);
        }
        public void end(boolean interrupted)    { hopperEncoder.setPosition(-6); }
        public boolean isFinished() { return (lastPos >= 5) && !firstCycle; }
        // public boolean isFinished() { return false; }
    }

    public Command homeCommand() {
        Command result = new HomingCommand();
        result.addRequirements(this);
        return result;
    }

    public void setPosition(double position) {
        hopperEncoder.setPosition(position);
    }

    public double getFF() { return -Math.sin( (hopperEncoder.getPosition()-20.0) *Math.PI/360)*Preferences.getDouble("FF Mult", 0.2); }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("Hopper Position", hopperEncoder.getPosition());
    }
}

package frc.robot.subsystems;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkBase.ControlType;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
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
            Constants.Hopper.maxHeight, Constants.Hopper.minHeight);

        this.hopperEncoder = hopperSpark.sparkEncode;
        this.hopperController = hopperSpark.sparkControl;
    }

    // TODO - code deploy and retract functions

    public Command deploy() {
        return Commands.none();
    }

    public Command retract() {
        return Commands.none();
    }

    public void setVoltage(double volts) {
        volts = MathUtil.clamp(volts, -5.0, 5.0);
        hopperController.setReference(volts, ControlType.kVoltage);
    }
}

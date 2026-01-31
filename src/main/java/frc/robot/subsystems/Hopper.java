package frc.robot.subsystems;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.math.controller.ProfiledPIDController;
import frc.lib.Items.SparkMax.SparkController;
import frc.lib.configs.Sparkmax.SparkControllerInfo;
import frc.robot.Constants;

public class Hopper {
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
}

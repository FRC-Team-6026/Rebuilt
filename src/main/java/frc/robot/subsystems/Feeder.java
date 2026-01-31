package frc.robot.subsystems;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkClosedLoopController;

import frc.lib.Items.SparkMax.SparkController;
import frc.lib.configs.Sparkmax.SparkControllerInfo;
import frc.robot.Constants;

public class Feeder {
    private SparkController feederSpark;
    private RelativeEncoder feederEncoder;
    private SparkClosedLoopController feederController;

    public Feeder() {
        this.feederSpark = new SparkController(Constants.Setup.feederSpark, new SparkControllerInfo().feeder());

        this.feederEncoder = feederSpark.sparkEncode;
        this.feederController = feederSpark.sparkControl;
    }
}

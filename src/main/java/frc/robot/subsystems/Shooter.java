package frc.robot.subsystems;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkClosedLoopController;

import frc.lib.Items.SparkMax.SparkController;
import frc.lib.configs.Sparkmax.SparkControllerInfo;
import frc.robot.Constants;

public class Shooter {
    private SparkController shooterSpark;
    private RelativeEncoder shooterEncoder;
    private SparkClosedLoopController shooterController;

    public Shooter() {
        this.shooterSpark = new SparkController(Constants.Setup.shooterSpark, new SparkControllerInfo().shooter());

        this.shooterEncoder = shooterSpark.sparkEncode;
        this.shooterController = shooterSpark.sparkControl;
    }
}

package frc.robot.subsystems;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkClosedLoopController;

import frc.lib.Items.SparkMax.SparkController;
import frc.lib.configs.Sparkmax.SparkControllerInfo;
import frc.robot.Constants;

public class Floor {
    private SparkController floorSpark;
    private RelativeEncoder floorEncoder;
    private SparkClosedLoopController floorController;

    public Floor() {
        this.floorSpark = new SparkController(Constants.Setup.floorSpark, new SparkControllerInfo().floor());

        this.floorEncoder = floorSpark.sparkEncode;
        this.floorController = floorSpark.sparkControl;
    }
}

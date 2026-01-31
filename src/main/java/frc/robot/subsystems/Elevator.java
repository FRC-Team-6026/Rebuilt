package frc.robot.subsystems;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkClosedLoopController;

import frc.lib.Items.SparkMax.SparkController;
import frc.lib.configs.Sparkmax.SparkControllerInfo;
import frc.robot.Constants;

public class Elevator {
    private SparkController elevatorSpark;
    private RelativeEncoder elevatorEncoder;
    private SparkClosedLoopController elevatorController;

    public Elevator() {
        this.elevatorSpark = new SparkController(Constants.Setup.elevatorSpark, new SparkControllerInfo().elevator(),
            Constants.Elevator.minPercent, Constants.Elevator.maxPercent,
            Constants.Elevator.maxHeight, Constants.Elevator.minHeight);

        this.elevatorEncoder = elevatorSpark.sparkEncode;
        this.elevatorController = elevatorSpark.sparkControl;
    }
}

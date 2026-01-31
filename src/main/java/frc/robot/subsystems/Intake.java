package frc.robot.subsystems;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkClosedLoopController;

import frc.lib.Items.SparkMax.SparkController;
import frc.lib.configs.Sparkmax.SparkControllerInfo;
import frc.robot.Constants;

public class Intake {
    private SparkController intakeSpark;
    private RelativeEncoder intakeEncoder;
    private SparkClosedLoopController intakeController;

    public Intake() {
        this.intakeSpark = new SparkController(Constants.Setup.intakeSpark, new SparkControllerInfo().intake());

        this.intakeEncoder = intakeSpark.sparkEncode;
        this.intakeController = intakeSpark.sparkControl;
    }
}

package frc.robot.subsystems;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkClosedLoopController;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.Items.SparkMax.SparkController;
import frc.lib.configs.Sparkmax.SparkControllerInfo;
import frc.robot.Constants;

public class Shooter extends SubsystemBase {
    private SparkController shooterSpark;
    private RelativeEncoder shooterEncoder;
    private SparkClosedLoopController shooterController;

    public Shooter() {
        this.shooterSpark = new SparkController(Constants.Setup.shooterSpark, new SparkControllerInfo().shooter());

        this.shooterEncoder = shooterSpark.sparkEncode;
        this.shooterController = shooterSpark.sparkControl;
    }

    // TODO - code start and stop functions
    public void start() { }

    public void stop() { }
}

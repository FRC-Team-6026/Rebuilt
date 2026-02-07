package frc.robot.subsystems;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkClosedLoopController;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.Items.SparkMax.SparkController;
import frc.lib.configs.Sparkmax.SparkControllerInfo;
import frc.robot.Constants;

public class Feeder extends SubsystemBase {
    private SparkController feederSpark;
    private RelativeEncoder feederEncoder;
    private SparkClosedLoopController feederController;

    // TODO - possibly combine into shooter subsystem, to make windup programming easier?
    public Feeder() {
        this.feederSpark = new SparkController(Constants.Setup.feederSpark, new SparkControllerInfo().feeder());

        this.feederEncoder = feederSpark.sparkEncode;
        this.feederController = feederSpark.sparkControl;
    }

    // TODO - code start and stop functions
    public void start() { }

    public void stop() { }
}

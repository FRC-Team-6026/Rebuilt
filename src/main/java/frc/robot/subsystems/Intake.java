package frc.robot.subsystems;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkClosedLoopController;

import edu.wpi.first.wpilibj.Preferences;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.Items.SparkMax.SparkController;
import frc.lib.configs.Sparkmax.SparkControllerInfo;
import frc.robot.Constants;

public class Intake extends SubsystemBase {
    private SparkController intakeSpark;
    private RelativeEncoder intakeEncoder;
    private SparkClosedLoopController intakeController;

    public Intake() {
        this.intakeSpark = new SparkController(Constants.Setup.intakeSpark, new SparkControllerInfo().intake());

        this.intakeEncoder = intakeSpark.sparkEncode;
        this.intakeController = intakeSpark.sparkControl;
    }

    // TODO - code start and stop functions
    public void start() { }

    public void stop() { }
}

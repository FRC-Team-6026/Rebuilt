package frc.robot.subsystems;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.SparkClosedLoopController;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.units.measure.Voltage;
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

    public void start() {
        setVoltage(Preferences.getDouble("Intake Volts", 0.5));
    }

    public void stop() { 
        setVoltage(0.0);
    }

    public void setVoltage(double voltage) {
        intakeController.setReference(voltage, SparkBase.ControlType.kVoltage);
    }
}

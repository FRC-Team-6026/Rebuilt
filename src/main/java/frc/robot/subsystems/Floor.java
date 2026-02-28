package frc.robot.subsystems;

import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.SparkClosedLoopController;

import edu.wpi.first.wpilibj.Preferences;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.Items.SparkMax.SparkController;
import frc.lib.configs.Sparkmax.SparkControllerInfo;
import frc.robot.Constants;

public class Floor extends SubsystemBase {
    private int FloorCalls;
    private SparkController floorSpark;
    private SparkClosedLoopController floorController;

    public Floor() {
        this.floorSpark = new SparkController(Constants.Setup.floorSpark, new SparkControllerInfo().floor());
        this.floorController = floorSpark.sparkControl;
    }

    public void start() {
        FloorCalls++;
        setVoltage(Preferences.getDouble("Floor Volts", 0.5));
    }

    public void stop() { 
        FloorCalls--;
        if (FloorCalls == 0) {
            setVoltage(0.0);
        }
    }

    public void setVoltage(double voltage) {
        floorController.setReference(voltage, SparkBase.ControlType.kVoltage);
    }
}

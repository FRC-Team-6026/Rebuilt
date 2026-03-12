package frc.robot.subsystems;

import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.SparkClosedLoopController;

import edu.wpi.first.wpilibj.Preferences;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.Items.SparkMax.SparkController;
import frc.lib.configs.Sparkmax.SparkControllerInfo;
import frc.robot.Constants;

public class Floor extends SubsystemBase {
    // private int FloorCalls;
    private SparkController floorSpark;
    private SparkClosedLoopController floorController;
    private boolean forward_requested = false;
    private boolean reverse_requested = false;

    public Floor() {
        this.floorSpark = new SparkController(Constants.Setup.floorSpark, new SparkControllerInfo().floor());
        this.floorController = floorSpark.sparkControl;
    }
    
    public void forward() {
        setVoltage(Preferences.getDouble("Floor Volts", 1));
        forward_requested = true;
    }
    
    public void reverse() {
        setVoltage(-Preferences.getDouble("Floor Volts", 1));
        reverse_requested = true;
    }

    public void stop() {
        forward_requested = false;
        reverse_requested = false;
        setVoltage(0.0);
    }

    public void stop(boolean forward) {
        if (forward) {
            forward_requested = false;
            if (reverse_requested)  setVoltage(-Preferences.getDouble("Floor Volts", 1));
            else                    setVoltage(0.0);
        }
        else {
            reverse_requested = false;
            if (forward_requested)  setVoltage(Preferences.getDouble("Floor Volts", 1));
            else                    setVoltage(0.0);
        }
    }

    public void setVoltage(double voltage) {
        floorController.setSetpoint(voltage, SparkBase.ControlType.kVoltage);
    }
}

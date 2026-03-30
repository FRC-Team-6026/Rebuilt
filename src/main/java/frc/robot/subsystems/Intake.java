package frc.robot.subsystems;

import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.SparkClosedLoopController;

import edu.wpi.first.wpilibj.Preferences;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.Items.Kraken.KrakenController;
import frc.lib.Items.SparkMax.SparkController;
import frc.lib.configs.Kraken.KrakenInfo;
import frc.lib.configs.Sparkmax.SparkControllerInfo;
import frc.robot.Constants;

public class Intake extends SubsystemBase {
    // TODO - cleanup commented code after kraken works
    // private SparkController intakeSpark;
    // private SparkClosedLoopController intakeController;
    private TalonFX intakeMotor;
    private KrakenController intakeController;
    private VoltageOut voltageControl;

    public Intake() {
        // this.intakeSpark = new SparkController(Constants.Setup.intakeSpark, new SparkControllerInfo().intake());
        // this.intakeController = intakeSpark.sparkControl;
        intakeController = new KrakenController(Constants.Setup.intakeSpark, new KrakenInfo().intake());
        voltageControl = new VoltageOut(0.0);
        intakeMotor = intakeController.motor;
    }

    public void start() {
        setVoltage(Preferences.getDouble("Intake Volts", 6));
    }

    public void stop() { 
        setVoltage(0.0);
    }

    public void setVoltage(double voltage) {
        intakeMotor.setControl(voltageControl.withOutput(voltage));
    }
}

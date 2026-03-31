package frc.robot.subsystems;

import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj.Preferences;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.Items.Kraken.KrakenController;
import frc.lib.configs.Kraken.KrakenInfo;
import frc.robot.Constants;

public class Intake extends SubsystemBase {
    private TalonFX intakeMotor;
    private KrakenController intakeController;
    private VoltageOut voltageControl;

    public Intake() {
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

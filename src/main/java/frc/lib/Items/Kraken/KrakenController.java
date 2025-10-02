package frc.lib.Items.Kraken;

import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.signals.InvertedValue;

// TODO - change this to a krakencontrollerinfo class
import frc.lib.configs.Sparkmax.SparkControllerInfo;

public class KrakenController {
    // TODO - all of it
    // https://v6.docs.ctr-electronics.com/en/latest/docs/api-reference/api-usage/configuration.html
    TalonFXConfiguration talonConfigs;
    // TODO - change this to a krakencontrollerinfo class
    public KrakenController(int canbusNumber, SparkControllerInfo Info) {
        talonConfigs = new TalonFXConfiguration();
        // Here's some example configuration to get started with.
        talonConfigs.withMotorOutput(new MotorOutputConfigs()
            .withInverted(InvertedValue.Clockwise_Positive)
        );
    }
}

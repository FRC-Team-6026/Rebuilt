package frc.lib.Items.Kraken;

import com.ctre.phoenix6.configs.AudioConfigs;
import com.ctre.phoenix6.configs.ClosedLoopGeneralConfigs;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import frc.lib.configs.Kraken.KrakenInfo;
// TODO - change this to a krakencontrollerinfo class
import frc.lib.configs.Sparkmax.SparkControllerInfo;

public class KrakenController {
    // TODO - all of it
    // https://v6.docs.ctr-electronics.com/en/latest/docs/api-reference/api-usage/configuration.html
    TalonFXConfiguration talonConfigs;
    // TODO - change this to a krakencontrollerinfo class
    public KrakenController(int canbusNumber, KrakenInfo Info) {
        talonConfigs = new TalonFXConfiguration();
        // Here's some example configuration to get started with.
        talonConfigs.withMotorOutput(new MotorOutputConfigs()
            .withInverted(Info.invert ? InvertedValue.CounterClockwise_Positive : InvertedValue.Clockwise_Positive)
            .withNeutralMode(Info.idleMode)
        ).withAudio(new AudioConfigs()
            .withBeepOnBoot(true)   // TODO - change beeping to false if this gets annoying
        ).withClosedLoopGeneral(new ClosedLoopGeneralConfigs()
            .withContinuousWrap(Info.continuousWrap)
        ).withFeedback(new FeedbackConfigs()
            .withRotorToSensorRatio(1)  // TODO - find a smooth way to tie angle motors to cancoder, or leave code in place?
            .withSensorToMechanismRatio(Info.posConversion)
        ).withCurrentLimits(new CurrentLimitsConfigs()
            .withSupplyCurrentLimit(Info.currentLim)
        );
    }
}

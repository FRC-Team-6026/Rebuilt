package frc.lib.Items.Kraken;

import com.ctre.phoenix6.configs.AudioConfigs;
import com.ctre.phoenix6.configs.ClosedLoopGeneralConfigs;
import com.ctre.phoenix6.configs.ClosedLoopRampsConfigs;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;

import frc.lib.configs.Kraken.KrakenInfo;

public class KrakenController {
    
    // https://v6.docs.ctr-electronics.com/en/latest/docs/api-reference/api-usage/configuration.html
    public TalonFXConfiguration talonConfigs;
    public TalonFX motor;

    public KrakenController(int canbusNumber, KrakenInfo Info) {
        motor = new TalonFX(canbusNumber);

        talonConfigs = new TalonFXConfiguration();
        talonConfigs
        .withAudio(new AudioConfigs()
            .withBeepOnBoot(true)   // TODO - change beeping to false if this gets annoying
        ).withSlot0(new Slot0Configs()
            .withKP(Info.pidList[0])
            .withKI(Info.pidList[1])
            .withKD(Info.pidList[2])
        ).withClosedLoopGeneral(new ClosedLoopGeneralConfigs()
            .withContinuousWrap(Info.continuousWrap)
        ).withClosedLoopRamps(new ClosedLoopRampsConfigs()
            .withDutyCycleClosedLoopRampPeriod(Info.rampRate)
            .withVoltageClosedLoopRampPeriod(Info.rampRate)     // TODO - change for SysID, when we do that
        ).withCurrentLimits(new CurrentLimitsConfigs()
            .withSupplyCurrentLimit(Info.currentLim)
        ).withFeedback(new FeedbackConfigs()
            .withRotorToSensorRatio(Info.RotorToSensorRatio)    // TODO - find a smooth way to tie angle motors to cancoder, or leave code in place?
            .withSensorToMechanismRatio(Info.SensorToMechanismRatio)
        ).withMotorOutput(new MotorOutputConfigs()
            .withInverted(Info.invert ? InvertedValue.CounterClockwise_Positive : InvertedValue.Clockwise_Positive)
            .withNeutralMode(Info.idleMode)
        );
        // If we use krakens for things other than the swerve drive, we should add software limit switch configuration.

        motor.getConfigurator().apply(talonConfigs);
    }
}

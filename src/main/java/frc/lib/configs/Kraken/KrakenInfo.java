package frc.lib.configs.Kraken;

import com.ctre.phoenix6.signals.NeutralModeValue;
import frc.lib.util.CANSparkMaxUtil.Usage;
import frc.robot.Constants.ConversionFactors;
import frc.robot.Constants.Electrical;
import frc.robot.Constants.IdleModes;
import frc.robot.Constants.PID;
import frc.robot.Constants.Setup;
import frc.robot.Constants.Usages;

// v5 CAN ID setting instructions: https://v5.docs.ctr-electronics.com/en/stable/ch08_BringUpCAN.html
// v6 CAN ID setting instructions: https://v6.docs.ctr-electronics.com/en/stable/docs/tuner/device-details-page.html

public class KrakenInfo {
    public Usage canbusUse;
    public int currentLim;
    public boolean invert;
    public boolean continuousWrap;
    // public boolean alternateAbsolute;    // is a throughbore plugged in as an absolute encoder?
    public NeutralModeValue idleMode;
    public double SensorToMechanismRatio;
    public double RotorToSensorRatio;
    public double[] pidList;
    public double voltageComp;
    public double rampRate = 0;

    public KrakenInfo drive() {
        canbusUse = Usages.driveUsage;
        currentLim = Electrical.driveCurrentLim;
        invert = Setup.driveInvert;
        idleMode = IdleModes.krakenDriveIdle;
        RotorToSensorRatio = ConversionFactors.driveKrakenRotorToSensorRatio;
        SensorToMechanismRatio = ConversionFactors.driveKrakenSensorToMechanismRatio;
        pidList = PID.drivePID;
        voltageComp = Electrical.voltageComp;
        continuousWrap = false;
        rampRate = 1.0; // seconds to reach max speed. TODO - tune ramp rate
        return this;
    }

    public KrakenInfo angle() {
        canbusUse = Usages.angleUsage;
        currentLim = Electrical.angleCurrentLim;
        idleMode = IdleModes.krakenAngleIdle;
        RotorToSensorRatio = ConversionFactors.angleKrakenRotorToSensorRatio;
        SensorToMechanismRatio = ConversionFactors.angleKrakenSensorToMechanismRatio;
        pidList = PID.anglePID;
        voltageComp = Electrical.voltageComp;
        continuousWrap = false; // flip back to true if we can figure out cancoder fusing, i think
        rampRate = 0.2; // seconds to reach max speed. TODO - tune ramp rate
        return this;
    }
}

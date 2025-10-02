package frc.lib.configs.Kraken;

import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import frc.lib.configs.Sparkmax.SparkControllerInfo;
import frc.lib.util.CANSparkMaxUtil.Usage;
import frc.robot.Constants.ConversionFactors;
import frc.robot.Constants.Electrical;
import frc.robot.Constants.IdleModes;
import frc.robot.Constants.PID;
import frc.robot.Constants.Setup;
import frc.robot.Constants.Usages;

public class KrakenInfo {
    public Usage canbusUse;
    public int currentLim;
    public boolean invert;
    public boolean alternateAbsolute;    // is a throughbore plugged in as an absolute encoder?
    public IdleMode idleMode;
    public double posConversion;
    public double velConversion;
    public double[] pidList;
    public double voltageComp;
    public double rampRate = 0;

    public SparkControllerInfo drive(){
        canbusUse = Usages.driveUsage;
        currentLim = Electrical.driveCurrentLim;
        invert = Setup.driveInvert;
        idleMode = IdleModes.driveIdle;
        posConversion = ConversionFactors.driveConversionPositionFactor;
        velConversion = ConversionFactors.driveConversionVelocityFactor;
        pidList = PID.drivePID;
        voltageComp = Electrical.voltageComp;
        return this;
    }

    public SparkControllerInfo angle(){
        canbusUse = Usages.angleUsage;
        currentLim = Electrical.angleCurrentLim;
        invert = Setup.angleInvert;
        alternateAbsolute = false;
        idleMode = IdleModes.angleIdle;
        posConversion = ConversionFactors.angleConversionPositionFactor;
        velConversion = ConversionFactors.angleConversionVelocityFactor;
        pidList = PID.anglePID;
        voltageComp = Electrical.voltageComp;
        return this;
    }
}

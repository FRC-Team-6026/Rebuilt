package frc.lib.configs.Sparkmax;

import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import frc.lib.util.CANSparkMaxUtil.Usage;
import frc.robot.Constants.*;

public class SparkControllerInfo {
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
    // public double maxmotionVel = 0, maxmotionAcc = 0;

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

    public SparkControllerInfo elevator(){
        canbusUse = Usages.elevatorUsage;
        currentLim = Electrical.elevatorLim;
        invert = Setup.elevatorInvert;
        alternateAbsolute = false;
        idleMode = IdleModes.elevatorIdle;
        posConversion = ConversionFactors.elevatorConversionPositionFactor;
        velConversion = ConversionFactors.elevatorConversionVelocityFactor;
        pidList = PID.elevatorPID;
        voltageComp = Electrical.voltageComp;
        // maxmotionVel = 10.0;
        // maxmotionAcc = 20.0;
        rampRate = 0.35;
        return this;
    }

    public SparkControllerInfo shooterWrist(){
        canbusUse = Usages.wristUsage;
        currentLim = Electrical.wristLim;
        invert = Setup.wristInvert;
        alternateAbsolute = true;
        idleMode = IdleModes.wristIdle;
        posConversion = ConversionFactors.wristConversionPositionFactor;
        velConversion = ConversionFactors.wristConversionVelocityFactor;
        pidList = PID.wristPID;
        voltageComp = Electrical.voltageComp;
        rampRate = 0.35;
        return this;
    }
} 
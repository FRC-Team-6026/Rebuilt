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
    public double velConversionFactor = 1.0/60.0;
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

    // TODO - double check all susbsystem values, please

    public SparkControllerInfo hopper(){
        canbusUse = Usages.hopperUsage;
        currentLim = Electrical.hopperLim;
        invert = Setup.hopperInvert;
        alternateAbsolute = false;
        idleMode = IdleModes.hopperIdle;
        posConversion = ConversionFactors.defaultConversionPositionFactor;
        velConversion = ConversionFactors.defaultConversionVelocityFactor;
        pidList = PID.hopperPID;
        voltageComp = Electrical.voltageComp;
        rampRate = 0.35;
        return this;
    }

    public SparkControllerInfo intake(){
        canbusUse = Usages.intakeUsage;
        currentLim = Electrical.intakeLim;
        invert = Setup.intakeInvert;
        alternateAbsolute = false;
        idleMode = IdleModes.intakeIdle;
        posConversion = ConversionFactors.defaultConversionPositionFactor;
        velConversion = ConversionFactors.defaultConversionVelocityFactor;
        pidList = PID.intakePID;
        voltageComp = Electrical.voltageComp;
        rampRate = 0.2;
        return this;
    }

    public SparkControllerInfo floor(){
        canbusUse = Usages.floorUsage;
        currentLim = Electrical.floorLim;
        invert = Setup.floorInvert;
        alternateAbsolute = false;
        idleMode = IdleModes.floorIdle;
        posConversion = ConversionFactors.defaultConversionPositionFactor;
        velConversion = ConversionFactors.defaultConversionVelocityFactor;
        pidList = PID.floorPID;
        voltageComp = Electrical.voltageComp;
        rampRate = 0.35;
        return this;
    }

    public SparkControllerInfo feeder(){
        canbusUse = Usages.feederUsage;
        currentLim = Electrical.feederLim;
        invert = Setup.feederInvert;
        alternateAbsolute = false;
        idleMode = IdleModes.feederIdle;
        posConversion = ConversionFactors.defaultConversionPositionFactor;
        velConversion = ConversionFactors.defaultConversionVelocityFactor;
        pidList = PID.feederPID;
        voltageComp = Electrical.voltageComp;
        rampRate = 0.35;
        return this;
    }

    public SparkControllerInfo shooter(){
        canbusUse = Usages.shooterUsage;
        currentLim = Electrical.shooterLim;
        invert = Setup.shooterInvert;
        alternateAbsolute = false;
        idleMode = IdleModes.shooterIdle;
        posConversion = ConversionFactors.defaultConversionPositionFactor;
        velConversion = ConversionFactors.defaultConversionVelocityFactor;
        pidList = PID.shooterPID;
        voltageComp = Electrical.voltageComp;
        rampRate = 0.35;
        return this;
    }

    public SparkControllerInfo elevator(){
        canbusUse = Usages.elevatorUsage;
        currentLim = Electrical.elevatorLim;
        invert = Setup.elevatorInvert;
        alternateAbsolute = false;
        idleMode = IdleModes.elevatorIdle;
        posConversion = ConversionFactors.defaultConversionPositionFactor;
        velConversion = ConversionFactors.defaultConversionVelocityFactor;
        pidList = PID.elevatorPID;
        voltageComp = Electrical.voltageComp;
        rampRate = 0.35;
        return this;
    }
} 
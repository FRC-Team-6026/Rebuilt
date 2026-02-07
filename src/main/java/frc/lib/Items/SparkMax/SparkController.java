package frc.lib.Items.SparkMax;

import com.revrobotics.spark.SparkLowLevel;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.SparkBase.ResetMode;

import frc.lib.configs.Sparkmax.SparkControllerInfo;
import frc.lib.util.CANSparkMaxUtil;
import frc.lib.util.CANSparkMaxUtil.Usage;

public class SparkController {
    public SparkMax spark;
    public RelativeEncoder sparkEncode;
    public AbsoluteEncoder sparkAbsoluteEncoder;
    public SparkClosedLoopController sparkControl;
    public final int canbusNumber;
    private final Usage canbusUse;
    private final int currentLim;
    private final boolean invert;
    private final IdleMode idleMode;
    private final double posConversion;
    private final double velConversion;
    private final double[] pidList;
    private final double voltageComp;
    private double max = 1;
    private double min = -1;
    public double fLim = 0;
    public double bLim = 0;
    public boolean fEnable = false;
    public boolean bEnable = false;
    public double rampRate = 0;
    // private double maxmotionVel, maxmotionAcc;


    /** Creates and Configures the Sparkmax Controller*/
    public SparkController(int canbusNumber, SparkControllerInfo Info){
        this.canbusNumber = canbusNumber;
        this.canbusUse = Info.canbusUse;
        this.currentLim = Info.currentLim;
        this.invert = Info.invert;
        this.idleMode = Info.idleMode;
        this.posConversion = Info.posConversion;
        this.velConversion = Info.velConversion;
        this.pidList = Info.pidList;
        this.voltageComp = Info.voltageComp;
        this.rampRate = Info.rampRate;
        spark = new SparkMax(canbusNumber, MotorType.kBrushless);
        sparkEncode = spark.getEncoder();
        if (Info.alternateAbsolute)
            sparkAbsoluteEncoder = spark.getAbsoluteEncoder();
        // this.maxmotionVel = Info.maxmotionVel;
        // this.maxmotionAcc = Info.maxmotionAcc;
        sparkControl = spark.getClosedLoopController();
        configureSpark();
    }

    /** Creates and Configures the Sparkmax Controller Note: Pass null to N/A fields
     * @param min minimum voltage from the PID controller
     * @param max maximum voltage from the PID controller
     * @param fLim soft limit for maximum forward position
     * @param bLim soft limit for maximum reverse position
     */
    public SparkController(int canbusNumber, SparkControllerInfo Info, Double min, Double max, Double fLim, Double bLim){
        this.canbusNumber = canbusNumber;
        this.canbusUse = Info.canbusUse;
        this.currentLim = Info.currentLim;
        this.invert = Info.invert;
        this.idleMode = Info.idleMode;
        this.posConversion = Info.posConversion;
        this.velConversion = Info.velConversion;
        this.pidList = Info.pidList;
        this.voltageComp = Info.voltageComp;
        this.rampRate = Info.rampRate;
        
        if(max != null){
            this.max = max;
        }
        if(min != null){
            this.min = min;
        }
        if(fLim != null){
            this.fLim = fLim;
            fEnable = true;
        }
        if(bLim != null){
            this.bLim = bLim;
            bEnable = true;
        }

        spark = new SparkMax(canbusNumber, SparkLowLevel.MotorType.kBrushless);
        sparkEncode = spark.getEncoder();
        if (Info.alternateAbsolute) {
            sparkAbsoluteEncoder = spark.getAbsoluteEncoder();
        } else {
            sparkAbsoluteEncoder = null;
        }
        sparkControl = spark.getClosedLoopController();
        configureSpark();
    }

    /* Sets and Flashes the Sparkmax to Passed States */
    public void configureSpark() {
        SparkMaxConfig config = new SparkMaxConfig();

        CANSparkMaxUtil.setCANSparkMaxBusUsage(spark, canbusUse);

        config.smartCurrentLimit(currentLim)
            .inverted(invert)
            .idleMode(idleMode)
            .voltageCompensation(voltageComp);

        config.encoder.velocityConversionFactor(velConversion)
                    .positionConversionFactor(posConversion);

        // if (this.maxmotionVel != 0) {
        //     config.closedLoop.maxMotion.positionMode(MAXMotionPositionMode.kMAXMotionTrapezoidal)
        //                 .maxVelocity(maxmotionVel)
        //                 .maxAcceleration(maxmotionAcc);
        // }

        if (this.rampRate != 0) {
            config.openLoopRampRate(rampRate)
                .closedLoopRampRate(rampRate);
        }

        config.closedLoop.pidf(pidList[0], pidList[1], pidList[2], pidList[3])
                    .outputRange(min, max)
                    .iZone(2.0)
                    .iMaxAccum(0.2);
        config.softLimit.forwardSoftLimit(fLim)
                    .reverseSoftLimit(bLim)
                    .forwardSoftLimitEnabled(fEnable)
                    .reverseSoftLimitEnabled(bEnable);
        
        spark.configure(config, ResetMode.kResetSafeParameters, null);
        spark.getEncoder().setPosition(0.0);

        /* 2024 -> 2025 import change. Configuration moved to a whole separate object.

        spark.restoreFactoryDefaults();
        CANSparkMaxUtil.setCANSparkMaxBusUsage(spark, canbusUse);
        spark.setSmartCurrentLimit(currentLim);
        spark.setInverted(invert);
        spark.setIdleMode(idleMode);
        sparkEncode.setVelocityConversionFactor(velConversion);
        sparkEncode.setPositionConversionFactor(posConversion);
        sparkControl.setP(pidList[0]);
        sparkControl.setI(pidList[1]);
        sparkControl.setD(pidList[2]);
        sparkControl.setFF(pidList[3]);
        spark.enableVoltageCompensation(voltageComp);
        sparkControl.setOutputRange(min, max);
        spark.setSoftLimit(SoftLimitDirection.kForward, ((float)fLim));
        spark.setSoftLimit(SoftLimitDirection.kReverse, ((float)bLim));
        spark.enableSoftLimit(SoftLimitDirection.kForward, fEnable);
        spark.enableSoftLimit(SoftLimitDirection.kReverse, bEnable);
        spark.burnFlash();
        sparkEncode.setPosition(0.0);
         */
    }
    
}

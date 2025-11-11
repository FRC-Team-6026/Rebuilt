package frc.lib.util;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkMaxConfig;

/** Sets motor usage for a Spark Max motor controller */
public class CANSparkMaxUtil {
  public enum Usage {
    kAll,
    kPositionOnly,
    kVelocityOnly,
    kMinimal
  };

  /**
   * This function allows reducing a Spark Max's CAN bus utilization by reducing the periodic status
   * frame period of nonessential frames from 20ms to 500ms.
   *
   * <p>See
   * https://docs.revrobotics.com/sparkmax/operating-modes/control-interfaces#periodic-status-frames
   * for a description of the status frames.
   * 
   * <p>This file has been edited to use the 2025 API. The above link is broken, but you can still
   * find this information here: https://docs.revrobotics.com/brushless/spark-max/control-interfaces
   *
   * @param motor The motor to adjust the status frame periods on.
   * @param usage The status frame feedack to enable. kAll is the default when a CANSparkMax is
   *     constructed.
   * @param enableFollowing Whether to enable motor following.
   */
  public static void setCANSparkMaxBusUsage(
      SparkMax motor, Usage usage, boolean enableFollowing) {
    SparkMaxConfig config = new SparkMaxConfig();

    if (enableFollowing) {
      config.signals.faultsPeriodMs(10);
    } else {
      config.signals.faultsPeriodMs(500);
    }

    if (usage == Usage.kAll) {
      config.signals.primaryEncoderVelocityPeriodMs(20);
      config.signals.primaryEncoderPositionPeriodMs(20);
      config.signals.analogPositionPeriodMs(50);
    } else if (usage == Usage.kPositionOnly) {
      config.signals.primaryEncoderVelocityPeriodMs(500);
      config.signals.primaryEncoderPositionPeriodMs(20);
      config.signals.analogPositionPeriodMs(500);
    } else if (usage == Usage.kVelocityOnly) {
      config.signals.primaryEncoderVelocityPeriodMs(20);
      config.signals.primaryEncoderPositionPeriodMs(500);
      config.signals.analogPositionPeriodMs(500);
    } else if (usage == Usage.kMinimal) {
      config.signals.primaryEncoderVelocityPeriodMs(500);
      config.signals.primaryEncoderPositionPeriodMs(500);
      config.signals.analogPositionPeriodMs(500);
    }
    

    /* 2024 -> 2025 import change. Configuration moved to a whole separate object.
    if (enableFollowing) {
      motor.setPeriodicFramePeriod(SparkLowLevel.PeriodicFrame.kStatus0, 10);
    } else {
      motor.setPeriodicFramePeriod(SparkLowLevel.PeriodicFrame.kStatus0, 500);
    }

    if (usage == Usage.kAll) {
      motor.setPeriodicFramePeriod(SparkLowLevel.PeriodicFrame.kStatus1, 20);
      motor.setPeriodicFramePeriod(SparkLowLevel.PeriodicFrame.kStatus2, 20);
      motor.setPeriodicFramePeriod(SparkLowLevel.PeriodicFrame.kStatus3, 50);
    } else if (usage == Usage.kPositionOnly) {
      motor.setPeriodicFramePeriod(SparkLowLevel.PeriodicFrame.kStatus1, 500);
      motor.setPeriodicFramePeriod(SparkLowLevel.PeriodicFrame.kStatus2, 20);
      motor.setPeriodicFramePeriod(SparkLowLevel.PeriodicFrame.kStatus3, 500);
    } else if (usage == Usage.kVelocityOnly) {
      motor.setPeriodicFramePeriod(SparkLowLevel.PeriodicFrame.kStatus1, 20);
      motor.setPeriodicFramePeriod(SparkLowLevel.PeriodicFrame.kStatus2, 500);
      motor.setPeriodicFramePeriod(SparkLowLevel.PeriodicFrame.kStatus3, 500);
    } else if (usage == Usage.kMinimal) {
      motor.setPeriodicFramePeriod(SparkLowLevel.PeriodicFrame.kStatus1, 500);
      motor.setPeriodicFramePeriod(SparkLowLevel.PeriodicFrame.kStatus2, 500);
      motor.setPeriodicFramePeriod(SparkLowLevel.PeriodicFrame.kStatus3, 500);
    }
     */
  }

  /**
   * This function allows reducing a Spark Max's CAN bus utilization by reducing the periodic status
   * frame period of nonessential frames from 20ms to 500ms.
   *
   * <p>See
   * https://docs.revrobotics.com/sparkmax/operating-modes/control-interfaces#periodic-status-frames
   * for a description of the status frames.
   * 
   * <p>This file has been edited to use the 2025 API. The above link is broken, but you can still
   * find this information here: https://docs.revrobotics.com/brushless/spark-max/control-interfaces
   *
   * @param motor The motor to adjust the status frame periods on.
   * @param usage The status frame feedack to enable. kAll is the default when a CANSparkMax is
   *     constructed.
   */
  public static void setCANSparkMaxBusUsage(SparkMax motor, Usage usage) {
    setCANSparkMaxBusUsage(motor, usage, false);
  }
}
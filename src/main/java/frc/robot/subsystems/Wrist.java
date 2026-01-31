package frc.robot.subsystems;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.SparkClosedLoopController;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.Items.SparkMax.SparkController;
import frc.lib.configs.Sparkmax.SparkControllerInfo;
import frc.robot.Constants;

// Left from previous year as example, not intended for 2026

public class Wrist extends SubsystemBase {

    private SparkController wristSpark;

    public RelativeEncoder wristEncoder;
    public AbsoluteEncoder wristAbsolute;

    public SparkClosedLoopController wristController;

    public ProfiledPIDController wristPID;

    public double currentVoltage;

    public double lastVelocity;

    public double lastVoltageAttempt;

    public Elevator s_Elevator;

    public Wrist() {
        this.wristSpark = new SparkController(Constants.Setup.wristSpark, new SparkControllerInfo().shooterWrist(),
            Constants.Wrist.minPercent, Constants.Wrist.maxPercent,
            Constants.Wrist.maximumAngle, Constants.Wrist.minimumAngle);
            // null, null);
        
        this.wristController = wristSpark.sparkControl;
        
        wristEncoder = wristSpark.sparkEncode;
        wristAbsolute = wristSpark.sparkAbsoluteEncoder;
        
        lastVelocity = 0;
        
        wristEncoder.setPosition(wristAbsolute.getPosition() * 360.0);
    }


    
    /** @return the angle, in degrees, that the wrist is actually at */
    public double getAngle() { return wristAbsolute.getPosition() * 360; }

    @Override
    public void periodic() {
        if (Math.abs(wristEncoder.getPosition() - (wristAbsolute.getPosition()*360.0)) > 0.05)
            wristEncoder.setPosition(wristAbsolute.getPosition() * 360.0);
        SmartDashboard.putNumber("Wrist Angle", wristAbsolute.getPosition()*360);
        // SmartDashboard.putNumber("Wrist Integrated Encoder", wristEncoder.getPosition());
    }

    public void setAngle(double angle) {
        angle = MathUtil.clamp(angle, Constants.Wrist.minimumAngle, Constants.Wrist.maximumAngle);
        wristController.setReference(angle, SparkBase.ControlType.kPosition);
    }

    public void setVoltage(double voltage) {
        voltage = MathUtil.clamp(voltage, -Constants.Wrist.maxVoltage, Constants.Wrist.maxVoltage);
        
        currentVoltage = voltage;
        if (s_Elevator.getHeight() >= 2 && getAngle() <= Constants.Elevator.selfDestructAngle + 2)
            voltage = MathUtil.clamp(voltage, 0, Constants.Wrist.maxVoltage);
        wristController.setReference(voltage, SparkBase.ControlType.kVoltage);
    }

    public void setDutyCycle(double percent) {
        wristPID.reset(getAngle());
        percent = percent/100;
        wristController.setReference(percent, SparkBase.ControlType.kDutyCycle);
    }
}

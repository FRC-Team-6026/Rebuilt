package frc.robot.subsystems;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.Items.SparkMax.SparkController;
import frc.lib.configs.Sparkmax.SparkControllerInfo;
import frc.robot.Constants;

// Left from previous year as example, not intended for 2026

public class Elevator extends SubsystemBase {

    public SparkController elevatorSpark1;
    public SparkController elevatorSpark2;
    
    public RelativeEncoder elevatorEncoder1;
    public RelativeEncoder elevatorEncoder2;

    public SparkClosedLoopController elevatorController1;
    public SparkClosedLoopController elevatorController2;

    public ProfiledPIDController elevProfiledPID;

    public Wrist wrist;
    private final double sdAngle = Constants.Elevator.selfDestructAngle;

    public Elevator(Wrist wrist) {
        this.elevatorSpark1 = new SparkController(Constants.Setup.elevatorSpark1, new SparkControllerInfo().elevator(),
            Constants.Elevator.minPercent, Constants.Elevator.maxPercent,
            Constants.Elevator.maxHeight, Constants.Elevator.minHeight);
        this.elevatorSpark2 = new SparkController(Constants.Setup.elevatorSpark2, new SparkControllerInfo().elevator(),
            Constants.Elevator.minPercent, Constants.Elevator.maxPercent,
            Constants.Elevator.maxHeight, Constants.Elevator.minHeight);
       
        SparkMaxConfig followerConfig = new SparkMaxConfig();
        followerConfig.follow(Constants.Setup.elevatorSpark1);
        this.elevatorSpark2.spark.configure(followerConfig, ResetMode.kNoResetSafeParameters, PersistMode.kPersistParameters);

        this.elevatorEncoder1 = elevatorSpark1.sparkEncode;
        this.elevatorEncoder2 = elevatorSpark2.sparkEncode;

        this.elevatorController1 = elevatorSpark1.sparkControl;
        this.elevatorController2 = elevatorSpark2.sparkControl;

        this.wrist = wrist;
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("Elevator Height", getHeight());
    }

    /** @return the height of the elevator carriage, in inches, from its fully lowered position */
    public double getHeight() {
        return elevatorEncoder1.getPosition();
    }

    public void setVoltage(Voltage voltage) {
        setVoltage(voltage.magnitude());
    }

    public void setVoltage(double voltage) {

        if(wrist.getAngle() < sdAngle) {
            return;
        }
        
        voltage = MathUtil.clamp(voltage, -Constants.Elevator.maxVoltage, Constants.Elevator.maxVoltage);

        elevatorController1.setReference(voltage, SparkBase.ControlType.kVoltage);
    }

    public void setDutyCycle(double percent){
        percent = percent/100;
        elevatorController1.setReference(percent, SparkBase.ControlType.kDutyCycle);
    }
}
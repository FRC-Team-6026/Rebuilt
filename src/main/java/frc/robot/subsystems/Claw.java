package frc.robot.subsystems;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.SparkClosedLoopController;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.Items.SparkMax.SparkController;
import frc.lib.configs.Sparkmax.SparkControllerInfo;
import frc.robot.Constants;

public class Claw extends SubsystemBase {

    private SparkController clawSpark;
    
    private RelativeEncoder clawEncoder;

    private SparkClosedLoopController clawController;

    public Claw() {
        this.clawSpark = new SparkController(Constants.Setup.clawSpark, new SparkControllerInfo().claw());
       
        this.clawEncoder = clawSpark.sparkEncode;

        this.clawController = clawSpark.sparkControl;
    }

    @Override
    public void periodic(){
        // SmartDashboard.putNumber("Claw Motor Velocity", clawEncoder.getVelocity());
    }

    public void setVoltage(double voltage){
        /*
        if(voltage < -Constants.Claw.maxVoltage){
            voltage = -Constants.Claw.maxVoltage;
        } else if (voltage > Constants.Claw.maxVoltage){
            voltage = Constants.Claw.maxVoltage;
        }
        */
        voltage = MathUtil.clamp(voltage, -Constants.Claw.maxVoltage, Constants.Claw.maxVoltage);

        clawController.setReference(voltage, SparkBase.ControlType.kVoltage);
    }

    public void setDutyCycle(double percent){
        percent = percent/100;
        clawController.setReference(percent, SparkBase.ControlType.kDutyCycle);
    }
}
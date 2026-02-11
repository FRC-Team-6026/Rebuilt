package frc.robot.subsystems;


import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkBase.ControlType;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import java.util.function.DoubleSupplier;
import frc.lib.Items.SparkMax.SparkController;
import frc.lib.configs.Sparkmax.SparkControllerInfo;
import frc.robot.Constants;

public class Shooter extends SubsystemBase {
    private SparkController shooterSpark;
    private RelativeEncoder shooterEncoder;
    private SparkClosedLoopController shooterController;
    
    private SparkController feederSpark;
    private RelativeEncoder feederEncoder;
    private SparkClosedLoopController feederController;

    public Shooter() {
        this.shooterSpark = new SparkController(Constants.Setup.shooterSpark, new SparkControllerInfo().shooter());
        this.feederSpark = new SparkController(Constants.Setup.feederSpark, new SparkControllerInfo().feeder());

        this.shooterEncoder = shooterSpark.sparkEncode;
        this.shooterController = shooterSpark.sparkControl;
        this.feederEncoder = feederSpark.sparkEncode;
        this.feederController = feederSpark.sparkControl;
    }

    public Command windup() { return new InstantCommand(() -> {
        shooterController.setReference(1.0, ControlType.kVoltage);
        feederController.setReference(0.0, ControlType.kVoltage);
    }, this);}

    public Command shoot(DoubleSupplier distance) { return Commands.run(() -> {
        // TODO - check units, refigure if robot params change
        double dist = distance.getAsDouble();

        // this approx formula is in meters/s, and is for specific robot settings
        // may need to adjust for wheel radius, speed imparted to ball, etc
        // double desiredSpeed = (dist-6)(21-dist)/22.2 + 9.3;
        // 4in diameter shooter wheels planned
        double desiredSpeed = 5;

        shooterController.setReference(desiredSpeed, ControlType.kVelocity);
        // Is 80% of value enough to start feeding stuff through?
        if (shooterEncoder.getVelocity() > 0.8*desiredSpeed)
            // This will enable, but not disable. Should be fine? Change if this is a problem
            feederController.setReference(1.0, ControlType.kVoltage);
    }, this);}

    public Command stop() { return new InstantCommand(() -> {
        shooterController.setReference(0.0, ControlType.kVoltage);
        feederController.setReference(0.0, ControlType.kVoltage);
    }, this);}
}

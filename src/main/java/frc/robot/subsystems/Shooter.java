package frc.robot.subsystems;


import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkBase.ControlType;

import edu.wpi.first.wpilibj.Preferences;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import java.util.function.DoubleSupplier;
import frc.lib.Items.SparkMax.SparkController;
import frc.lib.configs.Sparkmax.SparkControllerInfo;
import frc.robot.Constants;

public class Shooter extends SubsystemBase {
    private SparkController shooterSpark1;
    private SparkController shooterSpark2;
    private SparkController shooterSpark3;
    private RelativeEncoder shooterEncoder1;
    private SparkClosedLoopController shooterController1;
    
    private SparkController feederSpark;
    private RelativeEncoder feederEncoder;
    private SparkClosedLoopController feederController;

    public Shooter() {
        this.shooterSpark1 = new SparkController(Constants.Setup.shooterSpark1, new SparkControllerInfo().shooter());
        // this.shooterSpark2 = new SparkController(Constants.Setup.shooterSpark2, new SparkControllerInfo().shooter());
        // this.shooterSpark3 = new SparkController(Constants.Setup.shooterSpark3, new SparkControllerInfo().shooter());
        this.feederSpark = new SparkController(Constants.Setup.feederSpark, new SparkControllerInfo().feeder());

        this.shooterEncoder1 = shooterSpark1.sparkEncode;
        this.shooterController1 = shooterSpark1.sparkControl;
        this.feederEncoder = feederSpark.sparkEncode;
        this.feederController = feederSpark.sparkControl;
    }

    public void stop() {
        shooterController1.setReference(0.0, ControlType.kVoltage);
        // shooterController2.setReference(0.0, ControlType.kVoltage);
        // shooterController3.setReference(0.0, ControlType.kVoltage);
        feederController.setReference(0.0, ControlType.kVoltage);
    }

    public void windup() {
        // TODO - dial in minimum voltage. ideally this will be enough voltage for shooting at minimum distance
        shooterController1.setReference(Preferences.getDouble("Shooter Voltage", 0.5)/2.0, ControlType.kVoltage);
        // shooterController2.setReference(1.0, ControlType.kVoltage);
        // shooterController3.setReference(1.0, ControlType.kVoltage);
        feederController.setReference(0.0, ControlType.kVoltage);
    }

    public Command shootCommand() { return Commands.run(() -> {
        // TODO - check units, refigure if robot params change
        // double dist = distance.getAsDouble();

        // this approx formula is in meters/s, and is for specific robot settings
        // may need to adjust for wheel radius, speed imparted to ball, etc
        // double desiredBallSpeed = (dist-6)(21-dist)/22.2 + 9.3;
        // 4in diameter shooter wheels planned, so 4*pi circumference; 1 rev/s = 0.101*pi m/s (this is happening in conversion factors instead)
        // double desiredSpeed = desiredBallSpeed;
        // double desiredSpeed = 1; // expect values closer to 6, starting much lower for sanity checking first

        // shooterController1.setReference(desiredSpeed, ControlType.kVelocity);
        shooterController1.setReference(Preferences.getDouble("Shooter Voltage", 0.5), ControlType.kVoltage);
        // shooterController2.setReference(desiredSpeed, ControlType.kVelocity);
        // shooterController3.setReference(desiredSpeed, ControlType.kVelocity);
        // Is 80% of value enough to start feeding stuff through?
        // if (shooterEncoder1.getVelocity() > 0.8*desiredSpeed) {
            // This will enable, but not disable. Should be fine? Change if this is a problem
            // feederController.setReference(1.0, ControlType.kVoltage);
        // }
    }, this);}
}

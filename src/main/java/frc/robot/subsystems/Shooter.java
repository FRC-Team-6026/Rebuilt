package frc.robot.subsystems;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkBase.ControlType;

import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Preferences;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj.motorcontrol.Spark;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import java.util.function.DoubleSupplier;
import frc.lib.Items.SparkMax.SparkController;
import frc.lib.configs.Sparkmax.SparkControllerInfo;
import frc.robot.Constants;

public class Shooter extends SubsystemBase {
    
    // Switched shooters to use instancing. It should make our code much cleaner/shorter.
    private class ShooterMod {
        public SparkController spark;
        public RelativeEncoder encoder;
        public SparkClosedLoopController controller;

        public ShooterMod(int id) throws InterruptedException {
            this.spark = new SparkController(id, new SparkControllerInfo().shooter());
            if (this.spark.spark.getFaults().can) throw new InterruptedException();

            this.encoder = spark.sparkEncode;
            this.controller = spark.sparkControl;
        }
    }

    private ShooterMod[] s_mods;
    private Limelight limelight;
    
    private SparkController feederSpark;
    private SparkClosedLoopController feederController;

    private Alert sparkFail;
    private Alert limelightWarning;

    public Shooter(Limelight limelight) {
        s_mods = new ShooterMod[Constants.Setup.shooterSpark.length];
        for(int i = 0; i < Constants.Setup.shooterSpark.length; i++){
            this.sparkFail = new Alert("Failed to create Shooter Module (CAN ID " + Constants.Setup.shooterSpark[i]+ ")", AlertType.kError);
            try {
                s_mods[i] = new ShooterMod(Constants.Setup.shooterSpark[i]);
            } catch (InterruptedException e) {
                this.sparkFail.set(true);
            }
        }

        this.limelight = limelight;
        this.feederSpark = new SparkController(Constants.Setup.feederSpark, new SparkControllerInfo().feeder());
        this.feederController = feederSpark.sparkControl;
        this.limelightWarning = new Alert("Limelight can't determine distance to target", AlertType.kWarning);
    }

    public void stop() {
        for(ShooterMod mod : s_mods) {
            mod.controller.setReference(0.0, ControlType.kVoltage);
        }
        feederController.setReference(0.0, ControlType.kVoltage);
    }

    public void windup() {
        // TODO - dial in minimum voltage. ideally this will be enough voltage for shooting at minimum distance
        for(ShooterMod mod : s_mods) {
            mod.controller.setReference(Preferences.getDouble("Shooter Voltage", 0.5)/2.0, ControlType.kVoltage);
        }
        feederController.setReference(0.0, ControlType.kVoltage);
    }

    public Command shootCommand() { return Commands.run(() -> {
        // TODO - check units, refigure if robot params change
        // 4in diameter shooter wheels planned, so 4*pi circumference; 1 rev/s = 0.101*pi m/s (this is happening in conversion factors instead)
        
        // XXX - Testing if shooter works first. Switch to range finding later.
        for(ShooterMod mod : s_mods) {
            mod.controller.setReference(Preferences.getDouble("Shooter Voltage", 0.5), ControlType.kVoltage);
        }
        
        // double distance = limelight.getTZ();
        
        // if (distance == 0) {
        //     limelightWarning.set(true);
        //     return;
        // } else {
        //     limelightWarning.set(false);
        // }
        
        // double targetSpeed = (distance-6.0)*(21.0-distance)/22.2 + 9.3;

        // boolean atSpeed = true;
        // for (ShooterMod mod : s_mods) {
        //     mod.controller.setReference(targetSpeed, ControlType.kVelocity);
            
        //     if (mod.encoder.getVelocity() < 0.8 * targetSpeed) {
        //         atSpeed = false;
        //     }
        // }
        // if(atSpeed) {
        //     feederController.setReference(1.0, ControlType.kVoltage);
        // }
    }, this);}
}

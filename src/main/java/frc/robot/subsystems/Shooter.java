package frc.robot.subsystems;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkBase.ControlType;

import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Preferences;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
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
        
        int count = 0;
        ShooterMod[] mods = new ShooterMod[Constants.Setup.shooterSpark.length];
        
        for(int i = 0; i < Constants.Setup.shooterSpark.length; i++){
            this.sparkFail = new Alert("Failed to create Shooter Module (CAN ID " + Constants.Setup.shooterSpark[i]+ ")", AlertType.kError);
            try {
                mods[i] = new ShooterMod(Constants.Setup.shooterSpark[i]);
                count++;
            } catch (InterruptedException e) {
                this.sparkFail.set(true);
            }
        }

        int i = 0;
        s_mods = new ShooterMod[count];
        for(ShooterMod mod : mods) {
            if(mod != null) {
                s_mods[i] = mod;
                i++;
            }
        }

        this.limelight = limelight;
        this.feederSpark = new SparkController(Constants.Setup.feederSpark, new SparkControllerInfo().feeder());
        this.feederController = feederSpark.sparkControl;
        this.limelightWarning = new Alert("Limelight can't determine distance to target", AlertType.kWarning);
    }

    public void periodic() {
        for(ShooterMod mod : s_mods) {
            SmartDashboard.putNumber("Encoder test", mod.encoder.getVelocity());
        }
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
            mod.controller.setReference(Preferences.getDouble("Minimum Velocity (m/s)", 5.0), ControlType.kVelocity, ClosedLoopSlot.kSlot0, Preferences.getDouble("Minimum Velocity (m/s)", 5.0)*1.6);
        }
        feederController.setReference(0.0, ControlType.kVoltage);
    }

    public Command shootCommand() { return Commands.run(() -> {
        // 4in diameter shooter wheels planned, so 4*pi circumference; 1 rev/s = 0.101*pi m/s (this is happening in conversion factors instead)
        
        // 0.5969 meters to hub center from limelight
        double toHub = Math.cos(limelight.getYaw()) * 0.5969;
        double distance = toHub + limelight.getTZ();
        
        if (distance == 0) {
            limelightWarning.set(true);
            return;
        } else {
            limelightWarning.set(false);
        }
        
        // x2 for shooter wheel:fuel ratio
        double targetSpeed = 2 * ((distance-6.0)*(21.0-distance)/22.2 + 9.8);

        boolean atSpeed = true;
        for (ShooterMod mod : s_mods) {
            mod.controller.setReference(targetSpeed, ControlType.kVelocity, ClosedLoopSlot.kSlot0, targetSpeed*1.6);
            
            if (mod.encoder.getVelocity() < 0.9 * targetSpeed) {
                atSpeed = false;
            }
        }
        if(atSpeed) {
            feederController.setReference(Preferences.getDouble("Feeder Voltage", 0.5), ControlType.kVoltage);
        }
    }, this);}
}

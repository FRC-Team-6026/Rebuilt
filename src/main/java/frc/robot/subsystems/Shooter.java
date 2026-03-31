package frc.robot.subsystems;

import java.util.function.DoubleSupplier;

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
        public int id;

        public ShooterMod(int id) throws InterruptedException {
            this.spark = new SparkController(id, new SparkControllerInfo().shooter());
            if (this.spark.spark.getFaults().can) throw new InterruptedException();

            this.id = id;
            this.encoder = spark.sparkEncode;
            this.controller = spark.sparkControl;
        }
    }

    private ShooterMod[] s_mods;
    private Limelight limelight;
    private double distance;
    
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
            SmartDashboard.putNumber("Encoder test " + mod.id, mod.encoder.getVelocity());
        }
    }

    public void stop() {
        for(ShooterMod mod : s_mods) {
            mod.controller.setSetpoint(0.0, ControlType.kVoltage);
        }
        feederController.setSetpoint(0.0, ControlType.kVoltage);
    }

    public Command simpleShootCommand() { return Commands.startRun(
        () -> {
        for (ShooterMod mod : s_mods)
            mod.controller.setSetpoint(8, ControlType.kVoltage);},
        () -> {
        if (s_mods[0].encoder.getVelocity() > 7.0)
            feederController.setSetpoint(Preferences.getDouble("Feeder Volts", 0.5), ControlType.kVoltage);},
        this);
    }

    public Command shootCommand() {
        return shootCommand(() -> 0);
    }

    public Command shootCommand(DoubleSupplier extraVoltage) { return Commands.run(() -> {
        /* NEW distance calc velocity control */
        double toHub = Math.cos(limelight.getYaw()) * 0.5969;
        double tz = limelight.getTZ();

        if (tz == 0) { 
            distance = 2.8;
            limelightWarning.set(true);
        } 
        else { 
            distance = toHub + tz;
            limelightWarning.set(false); 
        }

        double targetSpeed = -0.235*(distance-16.3)*(3.0+distance);

        boolean atSpeed = true;
        for (ShooterMod mod : s_mods) {
            mod.controller.setSetpoint(
                targetSpeed,
                ControlType.kVelocity,
                ClosedLoopSlot.kSlot0,
                targetSpeed * Constants.Shooter.voltFactor
            );

            if (mod.encoder.getVelocity() < 0.92 * targetSpeed) atSpeed = false;
        }

        if(atSpeed) feederController.setSetpoint(Preferences.getDouble("Feeder Volts", 0.5), ControlType.kVoltage);
        else        feederController.setSetpoint(0.0, ControlType.kVoltage);
    }, this);}
}

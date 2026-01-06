package frc.lib.configs.Sparkmax;

import static edu.wpi.first.units.Units.Degrees;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.MagnetSensorConfigs;
import com.ctre.phoenix6.hardware.CANcoder;

import edu.wpi.first.units.measure.Angle;
import frc.lib.Items.Kraken.KrakenController;
import frc.lib.configs.Kraken.KrakenInfo;
import frc.robot.Constants;

public class SwerveModuleInfo {
    public int moduleNumber;
    public KrakenController drive;
    public KrakenController angle;
    public CANcoder cancoder;
    public double angleOffset;

    /**Requires the module to assign cancodes correctly
     * @param moduleNumber
     */

    public SwerveModuleInfo(int moduleNumber) {
        this.moduleNumber = moduleNumber;
        drive = new KrakenController(Constants.Setup.driveMotors[moduleNumber], new KrakenInfo().drive());
        angle = new KrakenController(Constants.Setup.angleMotors[moduleNumber], new KrakenInfo().angle(Constants.Setup.moduleCancoders[moduleNumber]));

        cancoder = new CANcoder(Constants.Setup.moduleCancoders[moduleNumber]);
        angleOffset = Constants.Setup.angleOffsets[moduleNumber];
        CANcoderConfiguration cancoderConfig = new CANcoderConfiguration();
        cancoderConfig
            .withMagnetSensor(new MagnetSensorConfigs()
                .withMagnetOffset(Degrees.of(angleOffset)));
        cancoder.getConfigurator().apply(cancoderConfig);
    }
}

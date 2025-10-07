package frc.lib.configs.Sparkmax;

import com.ctre.phoenix6.hardware.CANcoder;

import frc.lib.Items.Kraken.KrakenController;
import frc.lib.Items.SparkMax.SparkController;
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
        angle = new KrakenController(Constants.Setup.angleMotors[moduleNumber], new KrakenInfo().angle());

        // TODO - create KrakenControllerInfo.java
        // then switch over to:
        // drive = new KrakenControllerInfo(Constants.Setup.driveMotors[moduleNumber], etc etc);
        // angle = new KrakenControllerInfo(Constants.Setup.angleMotors[moduleNumber], etc etc);

        cancoder = new CANcoder(Constants.Setup.moduleCancoders[moduleNumber]);
        angleOffset = Constants.Setup.angleOffsets[moduleNumber];
    }
}

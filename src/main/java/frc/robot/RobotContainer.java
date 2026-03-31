// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.Collection;
import java.util.HashSet;
import java.util.List;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Preferences;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.subsystems.Floor;
import frc.robot.subsystems.Hopper;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Limelight;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Swerve;
import frc.lib.util.Elastic;
import frc.robot.commands.DefaultCommands.HopperDefault;
import frc.robot.commands.DefaultCommands.TeleopSwerve;

public class RobotContainer {

    public boolean angleConfigured = false;

    /* Controllers */
    private final XboxController driver = new XboxController(0);
    private final XboxController operator = new XboxController(1);

    /* Drive Controls */
    private final int translationAxis = XboxController.Axis.kLeftY.value;
    private final int strafeAxis = XboxController.Axis.kLeftX.value;
    private final int rotationAxis = XboxController.Axis.kRightX.value;

    /* Driver Buttons */

    /** Driver - Back (Minus) */
    private final JoystickButton zeroGyro = new JoystickButton(driver, XboxController.Button.kBack.value);
    /** Driver - Start (Plus) */
    private final JoystickButton robotCentricButton = new JoystickButton(driver, XboxController.Button.kStart.value);
    /** Driver - Y (X on our controller) */
    private final JoystickButton resetOdometry = new JoystickButton(driver, XboxController.Button.kY.value);
    /** Driver - A (B on our controller) */
    private final JoystickButton autoAimButton = new JoystickButton(driver, XboxController.Button.kA.value);

    private boolean robotCentric = false;

    // SysID buttons
    private final JoystickButton swerve_quasiF = new JoystickButton(driver, XboxController.Button.kA.value);
    private final JoystickButton swerve_quasiR = new JoystickButton(driver, XboxController.Button.kB.value);
    private final JoystickButton swerve_dynF = new JoystickButton(driver, XboxController.Button.kX.value);
    private final JoystickButton swerve_dynR = new JoystickButton(driver, XboxController.Button.kY.value);

    // private final JoystickButton sysid_on = new JoystickButton(driver, XboxController.Button.kLeftBumper.value);
    // private final JoystickButton sysid_off = new JoystickButton(driver, XboxController.Button.kRightBumper.value);


    /* Operator Buttons */

    // private final int elevatorAxis = XboxController.Axis.kRightY.value;
    private final int hopperAxis = XboxController.Axis.kLeftY.value;

    /** Operator - A (B on our controller) */
    private final JoystickButton retractButton = new JoystickButton(operator, XboxController.Button.kA.value);
    /** Operator - B (A on our controller) */
    private final JoystickButton deployButton = new JoystickButton(operator, XboxController.Button.kB.value);
    /** Operator - Y (X on our controller) */
    private final JoystickButton simpleShootButton = new JoystickButton(operator, XboxController.Button.kY.value);
    /** Operator - Left Bumper */
    private final JoystickButton floorReverseButton = new JoystickButton(operator, XboxController.Button.kLeftBumper.value);
    /** Operator - Right Bumper */
    private final JoystickButton floorForwardButton = new JoystickButton(operator, XboxController.Button.kRightBumper.value);

    /** Operator - Back Button */
    private final JoystickButton hopperHomingButton = new JoystickButton(operator, XboxController.Button.kBack.value);

    private final int intakeTrigger = XboxController.Axis.kLeftTrigger.value;
    /** Operator - Left Trigger */
    private final Trigger intakeButton = new Trigger(() -> operator.getRawAxis(intakeTrigger) > 0.2);

    private final int shootTrigger = XboxController.Axis.kRightTrigger.value;
    /** Operator - Right Trigger */
    private final Trigger shootButton = new Trigger(() -> operator.getRawAxis(shootTrigger) > 0.5);

    /* Subsystems */
    private final Swerve swerve = new Swerve();
    private final Limelight s_Limelight = new Limelight("limelight", swerve);

    private final Intake s_intake = new Intake();
    private final Hopper s_hopper = new Hopper();
    private final Floor s_floor = new Floor();
    private final Shooter s_shooter = new Shooter(s_Limelight);

    /* Robot Variables */
    private final SendableChooser<Command> autoChooser;

    public RobotContainer() {
        /* stops clogging our RIO's hard drive with talon logs */
        // SignalLogger.enableAutoLogging(false);

        /* Command Composition Definitions */
        Command complexAutoShoot = 
            s_shooter.shootCommand(() -> 0.0)
            .alongWith(new WaitCommand(1).andThen(
            new InstantCommand(() -> s_floor.forward()).andThen(
            new ConditionalCommand(
                s_hopper.tilt(),
                Commands.none(),
                () -> s_hopper.getPosition() > 60
            )))).finallyDo(() -> {
                s_shooter.stop();
                s_floor.stop(true);
                if (s_hopper.getPosition() > 60)
                    s_hopper.deploy().schedule();
                else
                    Commands.none();
            });

        /* PathPlanner named commands */
        NamedCommands.registerCommand("Shooter - Begin Firing", new InstantCommand(() -> s_floor.forward()).andThen(s_shooter.shootCommand()));
        NamedCommands.registerCommand("Shooter - Stop", new InstantCommand(() -> { s_shooter.stop(); s_floor.stop(); }, s_shooter));

        NamedCommands.registerCommand("Hopper - Home Position", s_hopper.homeCommand());
        NamedCommands.registerCommand("Hopper - Deploy", s_hopper.deploy());

        NamedCommands.registerCommand("Intake - Start", new InstantCommand(() -> s_intake.start()));
        NamedCommands.registerCommand("Intake - Stop", new InstantCommand(() -> s_intake.stop()));

        /* Preferences initialization */
        Collection<String> oldPrefs = Preferences.getKeys();
        Collection<String> newPrefs = new HashSet<String>() {
            {
                add("Aim Rotation Power");
                add("Intake Volts");
                add("Hopper Volts");
                add("Floor Volts");
                add("Hopper Deploy Target");
                // add("Hopper Trigger (Amps)");
                add("Minimum Velocity (V)");
                add("Feeder Volts");
                add("Shooter Mult");
            }
        };

        for (String pref : newPrefs) {
            Preferences.initDouble(pref, 0);
            oldPrefs.remove(pref);
        }
        for (String pref : oldPrefs) {
            Preferences.remove(pref);
        }

        SmartDashboard.putBoolean("Is Robot Centric", robotCentric);
        configureBindings();

        /**
         * Create and populate a sendable chooser with all PathPlannerAutos in the
         * project
         * This section of code is copied from AutoBuilder.buildAutoChooser and modified
         * to allow custom compound autos.
         */
        if (!AutoBuilder.isConfigured()) {
            throw new RuntimeException("AutoBuilder was not configured before attempting to build an auto chooser");
        }

        // autoChooser = AutoBuilder.buildAutoChooser();
        autoChooser = new SendableChooser<Command>();
        List<String> autoNames = AutoBuilder.getAllAutoNames();
        autoChooser.setDefaultOption("None", Commands.none());

        String[] ignoreAutos = Constants.AutoConfig.supportAutoList();
        for (String ignore : ignoreAutos) { autoNames.remove(ignore); }

        for (String autoName : autoNames) {
            PathPlannerAuto auto = new PathPlannerAuto(autoName);
            autoChooser.addOption(auto.getName(), auto);
        }

        /* Compound Auto Routines */
        /*
            * Example:
            * 
            * PathPlannerAuto driveForwardAuto = new PathPlannerAuto("Drive Forward");
            * PathPlannerAuto driveBackwardAuto = new PathPlannerAuto("Back Off Reef");
            * 
            * Command compound = Commands.sequence(
            * driveForwardAuto,
            * Commands.waitSeconds(1),
            * Commands.race(new AlignToReef(swerve, s_Limelight), Commands.waitSeconds(2)),
            * Commands.waitSeconds(1),
            * driveBackwardAuto
            * );
            * autoChooser.addOption("Test Compound", compound);
            * 
            */
        SmartDashboard.putData("Auto Mode", autoChooser);
    }

    private void configureBindings() {
        /* Driver Buttons */
        zeroGyro.onTrue(new InstantCommand(() -> swerve.zeroGyro()));
        robotCentricButton.onTrue(new InstantCommand(() -> {
            robotCentric = !robotCentric;
            SmartDashboard.putBoolean("Is Robot Centric", robotCentric);
        }));

        resetOdometry.onTrue(new InstantCommand(() -> swerve.resetToAbsolute()));

        /* Operator Buttons */
        intakeButton.onTrue(new InstantCommand(() -> { s_intake.start(); }));
        intakeButton.onFalse(new InstantCommand(() -> { s_intake.stop(); }));

        // TODO - change shoot button behavior (actually, we might not need the reverse pulsing...):
        // run shooter
        // turn on intake
        // run agitator(floor) anti-jam pulse cycle and repeat:
        //  - forward .5s, backwards .2s
        //  - and only run floor while feeder runs
        simpleShootButton.onTrue(s_shooter.simpleShootCommand());
        simpleShootButton.onFalse(new InstantCommand(() ->  s_shooter.stop(), s_shooter));

        shootButton.onTrue(
            s_shooter.shootCommand(() -> operator.getPOV()/180.0)
            .alongWith(new WaitCommand(1).andThen(
            new InstantCommand(() -> s_floor.forward()).andThen(
            new ConditionalCommand(
                s_hopper.tilt(),
                Commands.none(),
                () -> (s_hopper.getPosition() > 60) && shootButton.getAsBoolean()
            ))))
        );
        shootButton.onFalse(new InstantCommand(() ->  s_shooter.stop(), s_shooter)
            .alongWith(new InstantCommand(() -> s_floor.stop(true)).andThen(
            new ConditionalCommand(
                s_hopper.deploy(),
                Commands.none(),
                () -> s_hopper.getPosition() > 60
            )))
        );

        hopperHomingButton.onTrue(s_hopper.homeCommand());

        deployButton.onTrue(s_hopper.deploy());
        retractButton.onTrue(s_hopper.retract());
        floorReverseButton.onTrue(new InstantCommand(() -> s_floor.reverse()));
        floorReverseButton.onFalse(new InstantCommand(() -> s_floor.stop(false)));
        floorForwardButton.onTrue(new InstantCommand(() -> s_floor.forward()));
        floorForwardButton.onFalse(new InstantCommand(() -> s_floor.stop(true)));
    }

    public Command getAutonomousCommand() {
        return autoChooser.getSelected();
    }

    public void teleopInit() {
        swerve.resetToAbsolute();

        if (DriverStation.isFMSAttached()) {
            Elastic.selectTab("Teleop");
        }

        swerve.setDefaultCommand(
            new TeleopSwerve(
                swerve,
                s_Limelight,
                () -> -driver.getRawAxis(translationAxis),
                () -> -driver.getRawAxis(strafeAxis),
                () -> -driver.getRawAxis(rotationAxis),
                () -> autoAimButton.getAsBoolean(), // Activate limelight aiming functions
                () -> robotCentric
            )
        );

        s_hopper.setDefaultCommand(new HopperDefault(s_hopper, () -> operator.getRawAxis(hopperAxis)));
    }

    public void teleopExit() {
        swerve.removeDefaultCommand();
        s_hopper.removeDefaultCommand();

        // robot code crashed on teleop exit... commenting these out to see if they did it
        s_shooter.stop();
        s_intake.stop();
        s_floor.stop();
    }

    public void autoInit() {
        swerve.resetToAbsolute();

        if (DriverStation.isFMSAttached()) {
            Elastic.selectTab("Auto");
        }
    }

    public void autoExit() {
        s_shooter.stop();
        s_intake.stop();
        s_floor.stop();
    }

    public void testInit() {
        // Swerve SysID testing. Sets wheels forward and assigns each test to a button.
        swerve.resetToAbsolute();
        swerve.testInit().schedule();

        if (DriverStation.isFMSAttached()) {
            Elastic.selectTab(2);
        }

        // sysid_on.onTrue(new InstantCommand(() -> SignalLogger.start() ));
        // sysid_off.onTrue(new InstantCommand(() -> SignalLogger.stop() ));

        swerve_quasiF.onTrue(swerve.SysIDQuasiF().until(swerve_quasiF.negate()));
        swerve_quasiR.onTrue(swerve.SysIDQuasiR().until(swerve_quasiR.negate()));
        swerve_dynF.onTrue(swerve.SysIDDynF().until(swerve_dynF.negate()));
        swerve_dynR.onTrue(swerve.SysIDDynR().until(swerve_dynR.negate()));
    }

    public void testExit() {
        CommandScheduler.getInstance().cancelAll();
        CommandScheduler.getInstance().getActiveButtonLoop().clear();
    }
}
// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.ArrayList;
import java.util.List;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.wpilibj.Preferences;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.subsystems.Floor;
import frc.robot.subsystems.Hopper;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Limelight;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Swerve;
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

  // private final JoystickButton testButton =
  // new JoystickButton(driver, XboxController.Button.kDPadLeft.value);
  private final Trigger testButton =
  new Trigger(() -> (driver.getPOV() == 90));
  /** Driver - Back (Minus) */
  private final JoystickButton zeroGyro =
  new JoystickButton(driver, XboxController.Button.kBack.value);
  /** Driver - Start (Plus) */
  private final JoystickButton robotCentricButton =
  new JoystickButton(driver, XboxController.Button.kStart.value);
  /** Driver - Y (X on our controller) */
  private final JoystickButton resetOdometry = 
  new JoystickButton(driver, XboxController.Button.kY.value);
  /** Driver - A (B on our controller) */
  private final JoystickButton autoAimButton = 
  new JoystickButton(driver, XboxController.Button.kA.value);

  private boolean robotCentric = false;

  // SysID buttons
  // private final JoystickButton swerve_quasiF = new JoystickButton(driver, XboxController.Button.kA.value);
  // private final JoystickButton swerve_quasiR = new JoystickButton(driver, XboxController.Button.kB.value);
  // private final JoystickButton swerve_dynF = new JoystickButton(driver, XboxController.Button.kX.value);
  // private final JoystickButton swerve_dynR = new JoystickButton(driver, XboxController.Button.kY.value);

  /* Operator Buttons */
  
  private final int elevatorAxis = XboxController.Axis.kRightY.value;
  private final int hopperAxis = XboxController.Axis.kLeftY.value;

  /** Operator - A (B on our controller) */
  private final JoystickButton retractButton =
  new JoystickButton(operator, XboxController.Button.kA.value);
  /** Operator - B (A on our controller) */
  private final JoystickButton deployButton =
  new JoystickButton(operator, XboxController.Button.kB.value);
  /** Operator - Left Button */
  private final JoystickButton interruptButton =
  new JoystickButton(operator, XboxController.Button.kLeftBumper.value);
  
  private final int intakeTrigger = XboxController.Axis.kLeftTrigger.value;
  /** Operator - Left Trigger */
  private final Trigger intakeButton = new Trigger(() -> operator.getRawAxis(intakeTrigger) > 0.2);
  private final int shootTrigger = XboxController.Axis.kRightTrigger.value;
  /** Operator - Right Trigger, 20% */
  private final Trigger windupButton = new Trigger(() -> operator.getRawAxis(shootTrigger) > 0.2);
  /** Operator - Right Trigger, 80% */
  private final Trigger shootButton = new Trigger(() -> operator.getRawAxis(shootTrigger) > 0.8);

  /* Subsystems */
  // private final DigitalInput beambreak = new DigitalInput(Constants.Setup.beambreakID);
  private final Swerve swerve = new Swerve();
  private final Limelight s_Limelight = new Limelight("limelight", swerve);

  private final Intake s_intake = new Intake();
  private final Hopper s_hopper = new Hopper();
  private final Floor s_floor = new Floor();
  private final Shooter s_shooter = new Shooter(s_Limelight);

  /* Robot Variables */
  private final SendableChooser<Command> autoChooser;

  public RobotContainer() {

    /* Command Composition Definitions */
    
    /* PathPlanner named commands */
    NamedCommands.registerCommand("Limelight - Init Rotation", new InstantCommand(() -> {s_Limelight.configRotation(swerve.getPose().getRotation().getDegrees() - swerve.getGyro().getYaw());}));
    NamedCommands.registerCommand("Limelight - Config Rotation", new InstantCommand(() -> {angleConfigured = s_Limelight.configRotation(swerve);}).repeatedly().until(() -> angleConfigured));
    NamedCommands.registerCommand("Limelight - Update Pose", new InstantCommand(() -> s_Limelight.updatePose(swerve, true)));
    NamedCommands.registerCommand("Limelight - Update Pose MT1", new InstantCommand(() -> s_Limelight.updatePose(swerve, false)));

    configureBindings();

    /* Preferences initialization */
    // Preferences.removeAll();

    if (!Preferences.containsKey("AutoAimStrength")) {
      Preferences.initDouble("AutoAimStrength", 1.0);
    }
    if (!Preferences.containsKey("Intake Volts")) {
      Preferences.initDouble("Intake Volts", 0.5);
    }
    if (!Preferences.containsKey("Hopper Volts")) {
      Preferences.initDouble("Hopper Volts", 1);
    }
    if (!Preferences.containsKey("Floor Volts")) {
      Preferences.initDouble("Floor Volts", 0.5);
    }
    if (!Preferences.containsKey("FF Mult")) {
      Preferences.initDouble("FF Mult", 0.5);
    }
    if (!Preferences.containsKey("Hopper Deploy Target")) {
      Preferences.initDouble("Hopper Deploy Target", 60.0);
    }
    if (!Preferences.containsKey("Shooter Voltage")) {
      Preferences.initDouble("Shooter Voltage", 0.5);
    }

    /**
     * Create and populate a sendable chooser with all PathPlannerAutos in the project
     * This section of code is copied from AutoBuilder.buildAutoChooser and modified to allow custom compound autos.
     */
    if (!AutoBuilder.isConfigured()) {
      throw new RuntimeException(
          "AutoBuilder was not configured before attempting to build an auto chooser");
    }
    else {
      // autoChooser = AutoBuilder.buildAutoChooser(); // Default auto will be `Commands.none()`
      // SmartDashboard.putData("Auto Mode", autoChooser);
      
      autoChooser = new SendableChooser<Command>();

      List<String> autoNames = AutoBuilder.getAllAutoNames();
      PathPlannerAuto defaultOption = null;
      List<PathPlannerAuto> options = new ArrayList<>();

      String defaultAutoName = Constants.AutoConfig.defaultAutoName();
      String[] ignoreAutos = Constants.AutoConfig.supportAutoList();
      for (String ignore : ignoreAutos) { autoNames.remove(ignore); }

      for (String autoName : autoNames) {
        PathPlannerAuto auto = new PathPlannerAuto(autoName);

        if (!defaultAutoName.isEmpty() && defaultAutoName.equals(autoName))
                  { defaultOption = auto; }
        else      { options.add(auto); }
      }

      if (defaultOption == null) {
        autoChooser.setDefaultOption("None", Commands.none());
      } else {
        autoChooser.setDefaultOption(defaultOption.getName(), defaultOption);
        autoChooser.addOption("None", Commands.none());
      }

      for (PathPlannerAuto auto : options ) 
        {autoChooser.addOption(auto.getName(), auto);}
      
      /* Compound Auto Routines */
      /* Example:

      PathPlannerAuto driveForwardAuto = new PathPlannerAuto("Drive Forward");
      PathPlannerAuto driveBackwardAuto = new PathPlannerAuto("Back Off Reef");

      Command compound = Commands.sequence(
        driveForwardAuto,
        Commands.waitSeconds(1),
        Commands.race(new AlignToReef(swerve, s_Limelight), Commands.waitSeconds(2)),
        Commands.waitSeconds(1),
        driveBackwardAuto
      );
      autoChooser.addOption("Test Compound", compound);
      
      */

      SmartDashboard.putData("Auto Mode", autoChooser);
      SmartDashboard.putBoolean("Is Robot Centric", robotCentric);
      
      // s_Limelight.configRotation(swerve);
    }
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
    testButton.onChange(new InstantCommand(() -> SmartDashboard.putBoolean("DPad Pressed", testButton.getAsBoolean())));

    intakeButton.onTrue(new InstantCommand(() -> {s_intake.start(); s_floor.start();}));
    intakeButton.onFalse(new InstantCommand(() -> {s_intake.stop(); s_floor.stop();}));

    shootButton.onTrue(new InstantCommand(() -> s_floor.start()).andThen(s_shooter.shootCommand()));
    shootButton.onFalse(new InstantCommand(() -> {s_shooter.windup(); s_floor.stop();}, s_shooter));
    windupButton.onTrue(new InstantCommand(() -> s_shooter.windup(), s_shooter));
    windupButton.onFalse(new InstantCommand(() -> s_shooter.stop(), s_shooter));

    deployButton.onTrue(s_hopper.deploy());
    retractButton.onTrue(s_hopper.retract());
 }

 public Command getAutonomousCommand() {
   return autoChooser.getSelected();
 }

  public void teleopInit() {
    swerve.resetToAbsolute();

    swerve.setDefaultCommand(
      new TeleopSwerve(
        swerve,
        s_Limelight,
        () -> -driver.getRawAxis(translationAxis),
        () -> -driver.getRawAxis(strafeAxis),
        () -> -driver.getRawAxis(rotationAxis),
        // () -> (autoDrive.getAsBoolean() ? s_Limelight.getTZ() : -driver.getRawAxis(translationAxis)),
        // () -> (autoDrive.getAsBoolean() ? s_Limelight.getTX() : -driver.getRawAxis(strafeAxis)),
        // () -> -driver.getRawAxis(rotationAxis), // To enable the autoaim button again, comment this line and uncomment the line below
        () -> autoAimButton.getAsBoolean(),
        () -> robotCentric));
        // () -> (autoDrive.getAsBoolean() ? true : robotCentric)));

    s_hopper.setDefaultCommand(
      new HopperDefault(s_hopper, () -> operator.getRawAxis(hopperAxis))
    );
  }

  public void teleopExit() {
    swerve.removeDefaultCommand();
    s_hopper.removeDefaultCommand();
  }

  public void autoInit() {
    swerve.resetToAbsolute();
  }

  public void testInit() {
    // Swerve SysID testing. Sets wheels forward and assigns each test to a button.
    swerve.resetToAbsolute();
    swerve.testInit().schedule();

    // swerve_quasiF.onTrue( swerve.SysIDQuasiF().until( swerve_quasiF.negate()));
    // swerve_quasiR.onTrue( swerve.SysIDQuasiR().until( swerve_quasiR.negate()));
    // swerve_dynF.onTrue(   swerve.SysIDDynF().until(   swerve_dynF.negate()));
    // swerve_dynR.onTrue(   swerve.SysIDDynR().until(   swerve_dynR.negate()));
    
  }

  public void testExit() {
    swerve.getCurrentCommand().cancel();

    // This command clears all button bindings. I think if we switch straight from test mode to
    // teleop, we won't have any buttons... but we'd still have default commands.
    CommandScheduler.getInstance().getActiveButtonLoop().clear();
  }
}
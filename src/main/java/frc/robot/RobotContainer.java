// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.commands.*;
import frc.robot.subsystems.*;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  
  /* Subsystems */
  public final Swerve s_Swerve = new Swerve();
  public final Arm arm = new Arm();
  public final Intake intake = new Intake();
  public final Launcher launcher = new Launcher();
  public final Feed feed = new Feed();
  // public final Climber climber = new Climber();

  /* Controllers */
  private final Joystick driver = new Joystick(0);
  private final Joystick operator = new Joystick(1);

  /* Drive Controls */
  private final int translationAxis = XboxController.Axis.kLeftY.value;
  private final int strafeAxis = XboxController.Axis.kLeftX.value;
  private final int rotationAxis = XboxController.Axis.kRightX.value;

  /* Driver Buttons */
  private final JoystickButton zeroGyro =
      new JoystickButton(driver, XboxController.Button.kY.value);
  private final JoystickButton robotCentric =
      new JoystickButton(driver, XboxController.Button.kLeftBumper.value);
  private final JoystickButton lock = new JoystickButton(driver, 3);
  private final JoystickButton slow = new JoystickButton(driver, 1);
  // private final JoystickButton resetToLimelight = new JoystickButton(driver, 2);
  // private final JoystickButton climb = new JoystickButton(driver, 4);
  // private final JoystickButton pickup = new JoystickButton(driver, 5);
  // private final JoystickButton launch = new JoystickButton(driver, 6);
  // private final JoystickButton preset1 = new JoystickButton(operator, 2);
  // private final JoystickButton preset2 = new JoystickButton(operator, 3);
  // private final JoystickButton preset3 = new JoystickButton(operator, 4);
  // private final JoystickButton subwooferArm = new JoystickButton(driver, 2);

  /** The container for the robot. Contains subsystems, OI devices, and commands. */

  private SendableChooser<Command> autoChooser;

  public RobotContainer() {
    s_Swerve.setDefaultCommand(
        new TeleopSwerve(
            s_Swerve,
            () -> driver.getRawAxis(translationAxis),
            () -> driver.getRawAxis(strafeAxis),
            () -> -driver.getRawAxis(2),
            () -> robotCentric.getAsBoolean()));
    
    // s_Swerve.setDefaultCommand(
    //     new NaiveAutoAlignCMD(
    //         s_Swerve,
    //         () -> -driver.getRawAxis(translationAxis),
    //         () -> driver.getRawAxis(strafeAxis),
    //         () -> robotCentric.getAsBoolean(),
    //         new int[]{5}));

    // arm.setDefaultCommand(
    //   new ArmJoystickCMD(
    //     arm,
    //     () -> operator.getRawAxis(1)
    //   )
    // );

    launcher.setDefaultCommand(
      new LauncherTriggerCMD(
        launcher,
      () -> operator.getRawAxis(3)
      )
    );

    feed.setDefaultCommand(
        new FeedTriggerCMD(
        feed,
      () -> operator.getRawAxis(5),
      () -> operator.getRawAxis(3)
      )
    );

    intake.setDefaultCommand(
      new IntakeTriggerCMD(
        intake,
        () -> operator.getRawAxis(5)
      )
    );
    
    //Register Named Commands - Temporary
    //NamedCommands.registerCommand("test", new Lock(s_Swerve));
    NamedCommands.registerCommand("Lock", new LockCMD(s_Swerve));

    // Configure the button bindingszeroGyro
    configureButtonBindings();
    

    autoChooser = AutoBuilder.buildAutoChooser();
    SmartDashboard.putData("Select Auto", autoChooser);
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
    /* Driver Buttons */
    zeroGyro.onTrue(new InstantCommand(() -> s_Swerve.zeroGyro()));
    lock.onTrue(new LockCMD(s_Swerve));
    slow.onTrue(new SetSwerveSpeedCMD(s_Swerve, 0.5)).onFalse(new SetSwerveSpeedCMD(s_Swerve,1));
    // resetToLimelight.onTrue(new InstantCommand(() -> s_Swerve.resetOdometryToLimelight()));
    // climb.onTrue(new InstantCommand(() -> climber.toggle()));
    // pickup.toggleOnTrue(new IntakeCMD(intake, arm));
    // launch.toggleOnTrue(new ShootCMD(launcher, feed, intake, arm));
    // preset1.onTrue(new SetArmPositionCMD(arm, Constants.OperatorConstants.intakePos));
    // preset2.onTrue(new SetArmPositionCMD(arm, Constants.OperatorConstants.nearLaunchPosition));
    // preset3.onTrue(new SetArmPositionCMD(arm, Constants.OperatorConstants.farLaunchPosition));
    // subwooferArm.onTrue(new SetArmPositionCMD(arm, Constants.OperatorConstants.ampLaunchPosition));
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An ExampleCommand will run in autonomous
    //return new exampleAuto(s_Swerve);
    return autoChooser.getSelected();
  }
}
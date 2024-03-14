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
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.commands.*;
import frc.robot.commands.manual.ArmJoystickCMD;
import frc.robot.commands.manual.FeedTriggerCMD;
import frc.robot.commands.manual.IntakeTriggerCMD;
import frc.robot.commands.manual.LauncherTriggerCMD;
import frc.robot.commands.manual.TeleopSwerve;
import frc.robot.commands.presets.*;
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

  /* Driver Buttons */
  private final JoystickButton zeroGyro =
      new JoystickButton(driver, Constants.ControllerMap.y);
  private final JoystickButton robotCentric =
      new JoystickButton(driver, Constants.ControllerMap.LB);
  private final JoystickButton lock = new JoystickButton(driver, Constants.ControllerMap.x);
  private final JoystickButton slow = new JoystickButton(driver, Constants.ControllerMap.a);
  // private final JoystickButton resetToLimelight = new JoystickButton(driver, Constants.ControllerMap.b);
  // private final JoystickButton climb = new JoystickButton(driver, Constants.ControllerMap.y);
  // private final JoystickButton pickup = new JoystickButton(driver, Constants.ControllerMap.LB);
  // private final JoystickButton launch = new JoystickButton(driver, Constants.ControllerMap.RB);
  // private final JoystickButton preset1 = new JoystickButton(operator, Constants.ControllerMap.b);
  // private final JoystickButton preset2 = new JoystickButton(operator, Constants.ControllerMap.x);
  // private final JoystickButton preset3 = new JoystickButton(operator, Constants.ControllerMap.y);

  /** The container for the robot. Contains subsystems, OI devices, and commands. */

  private SendableChooser<Command> autoChooser;
  
  public RobotContainer() {
    SmartDashboard.putBoolean("seneca auto", false);
    
    s_Swerve.setDefaultCommand(
        new TeleopSwerve(
            s_Swerve,
            () -> driver.getRawAxis(Constants.ControllerMap.leftStickY),
            () -> driver.getRawAxis(Constants.ControllerMap.leftStickX),
            () -> -driver.getRawAxis(Constants.ControllerMap.rightStickX),
            () -> robotCentric.getAsBoolean()));
    
    // s_Swerve.setDefaultCommand(
    //     new NaiveAutoAlignCMD(
    //         s_Swerve,
    //         () -> -driver.getRawAxis(translationAxis),
    //         () -> driver.getRawAxis(strafeAxis),
    //         () -> robotCentric.getAsBoolean(),
    //         new int[]{5}));

    arm.setDefaultCommand(
      new ArmJoystickCMD(
        arm,
        () -> operator.getRawAxis(Constants.ControllerMap.leftStickY)
      )
    );

    launcher.setDefaultCommand(
      new LauncherTriggerCMD(
        launcher,
      () -> operator.getRawAxis(Constants.ControllerMap.leftTrigger)
      )
    );

    feed.setDefaultCommand(
        new FeedTriggerCMD(
        feed,
      () -> operator.getRawAxis(Constants.ControllerMap.rightStickY),
      () -> operator.getRawAxis(Constants.ControllerMap.rightStickY)
      )
    );

    intake.setDefaultCommand(
      new IntakeTriggerCMD(
        intake,
        () -> operator.getRawAxis(Constants.ControllerMap.rightStickY)
      )
    );
    
    //Register Named Commands - Temporary
    //NamedCommands.registerCommand("test", new Lock(s_Swerve));
    NamedCommands.registerCommand("Lock", new LockCMD(s_Swerve));
    NamedCommands.registerCommand("Shoot", new ShootAndStopCMD(launcher, feed, intake, arm));
    NamedCommands.registerCommand("Pickup", new PrintCommand("Placeholder"));

    // Configure the button bindingszeroGyro
    configureButtonBindings();
    

    autoChooser = AutoBuilder.buildAutoChooser();
    SmartDashboard.putData("Select Auto", autoChooser);

    SmartDashboard.putNumber("Wait Time", 0);
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
    slow.onTrue(new SetSwerveSpeedCMD(s_Swerve, 0.5)).onFalse(new SetSwerveSpeedCMD(s_Swerve,3));
    // resetToLimelight.onTrue(new InstantCommand(() -> s_Swerve.resetOdometryToLimelight()));
    // climb.onTrue(new InstantCommand(() -> climber.toggle()));
    // pickup.toggleOnTrue(new IntakeCMD(intake, arm));
    // launch.toggleOnTrue(new ShootCMD(launcher, feed, intake, arm));
    // preset1.onTrue(new SetArmPositionCMD(arm, Constants.OperatorConstants.intakePos));
    // preset2.onTrue(new SetArmPositionCMD(arm, Constants.OperatorConstants.nearLaunchPosition));
    // preset3.onTrue(new SetArmPositionCMD(arm, Constants.OperatorConstants.farLaunchPosition));
    // subwooferArm.onTrue(new SetArmPositionCMD(arm, Constants.OperatorConstants.ampLaunchPosition));
  }

  public void teleopInit(){
    s_Swerve.resetEncoders();
    //REMEMBER THIS LINE
    s_Swerve.zeroGyro();
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An ExampleCommand will run in autonomous
    //return new exampleAuto(s_Swerve);
    //return new DriveStraightCMD(s_Swerve);
    // if(SmartDashboard.getBoolean("seneca auto", false)){
    //   return new DriveStraightCMD(s_Swerve);
    // }else{
    //   return new WaitCommand(1);
    // }
    return autoChooser.getSelected();
  }
}
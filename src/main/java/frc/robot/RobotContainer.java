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
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
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
  //public final Climber climber = new Climber();
  public final BeamBreak beamBreak = new BeamBreak();

  /* Controllers */
  private final Joystick driver = new Joystick(0);
  private final Joystick operator = new Joystick(1);

//driver
//lt intake
//rt shoot
//lb slow mode
//rb outtake
//x is lock
//y is speaker
//b is amp
//a is intake arm position

//operator
//arm as is, feed/intake as is
//rt launch reverse
//lt launch normal
//y zero gyro
//a robotcentrix
//b climb

  /* Driver Buttons */
  private final JoystickButton zeroGyro = new JoystickButton(operator, Constants.ControllerMap.y);
  private final JoystickButton robotCentric = new JoystickButton(operator, Constants.ControllerMap.a);
  private final JoystickButton climb = new JoystickButton(operator, Constants.ControllerMap.b);

  private final JoystickButton slow = new JoystickButton(driver, Constants.ControllerMap.LB);
  private final JoystickButton pickup = new JoystickButton(driver, Constants.ControllerMap.leftTrigger);

  private final JoystickButton outtake = new JoystickButton(driver, Constants.ControllerMap.RB);
  
  private final JoystickButton pickupPosition = new JoystickButton(driver, Constants.ControllerMap.b);
  private final JoystickButton ampLaunch = new JoystickButton(driver, Constants.ControllerMap.a);
  private final JoystickButton speakerLaunch = new JoystickButton(driver, Constants.ControllerMap.y);

  private final JoystickButton rawLaunch  = new JoystickButton(driver, Constants.ControllerMap.rightTrigger);
  private final JoystickButton lock = new JoystickButton(driver, Constants.ControllerMap.x);

  private final JoystickButton ampPosition  = new JoystickButton(operator, Constants.ControllerMap.rightTrigger);
  private final JoystickButton speakerPosition = new JoystickButton(operator, Constants.ControllerMap.leftTrigger);

  /** The container for the robot. Contains subsystems, OI devices, and commands. */

  private SendableChooser<Command> autoChooser;
  
  public RobotContainer() {
    
    // s_Swerve.setDefaultCommand(
    //     new NaiveAutoAlignCMD(
    //         s_Swerve,
    //         () -> -driver.getRawAxis(translationAxis),
    //         () -> driver.getRawAxis(strafeAxis),
    //         () -> robotCentric.getAsBoolean(),
    //         new int[]{5}));

    s_Swerve.setDefaultCommand(
        new TeleopSwerve(
            s_Swerve,
            () -> driver.getRawAxis(Constants.ControllerMap.leftStickY),
            () -> driver.getRawAxis(Constants.ControllerMap.leftStickX),
            () -> driver.getRawAxis(Constants.ControllerMap.rightStickX),
            () -> robotCentric.getAsBoolean()));

    arm.setDefaultCommand(
      new ArmJoystickCMD(
        arm,
        () -> -operator.getRawAxis(Constants.ControllerMap.leftStickY)
      )
    );

    launcher.setDefaultCommand(
      new LauncherTriggerCMD(
        launcher,
        () -> operator.getRawAxis(Constants.ControllerMap.rightStickX),
        () -> operator.getRawAxis(Constants.ControllerMap.rightTrigger)
      )
    );

    feed.setDefaultCommand(
        new FeedTriggerCMD(
        feed,
        () -> operator.getRawAxis(Constants.ControllerMap.rightStickX),
        () -> -operator.getRawAxis(Constants.ControllerMap.rightStickY)
      )
    );

    intake.setDefaultCommand(
      new IntakeTriggerCMD(
        intake,
        () -> -operator.getRawAxis(Constants.ControllerMap.rightStickY)
      )
    );
    
    //Register Named Commands - Temporary
    NamedCommands.registerCommand("Lock", new LockCMD(s_Swerve));
    NamedCommands.registerCommand("Shoot", new SpeakerLaunchCMD(launcher, feed, intake, arm));
    NamedCommands.registerCommand("Pickup", new IntakeCMD(intake, arm, feed));
    // Configure the button bindingszeroGyro
    configureButtonBindings();
    

    autoChooser = AutoBuilder.buildAutoChooser();
    SmartDashboard.putData("Select Auto", autoChooser);
    SmartDashboard.putBoolean("Auto Drive", false);
    SmartDashboard.putBoolean("Auto Shoot", false);
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
    slow.onTrue(new SetSwerveSpeedCMD(s_Swerve, 0.3)).onFalse(new SetSwerveSpeedCMD(s_Swerve,1));
    ampLaunch.onTrue(new AmpLaunchCMD(launcher, feed, intake, arm));
    speakerLaunch.onTrue(new SpeakerLaunchCMD(launcher, feed, intake, arm));
    pickupPosition.onTrue(new SetArmPositionCMD(arm, Constants.OperatorConstants.intakePosition));
    //climb.onTrue(new InstantCommand(()-> climber.toggle()));
    pickup.onTrue(new IntakeCMD(intake, arm, feed));
    outtake.onTrue(new OuttakeCMD(intake, feed));
    rawLaunch.onTrue(new SpeakerLaunchWithoutArmCMD(launcher, feed, intake));

    ampPosition.onTrue(new SetArmPositionCMD(arm, Constants.OperatorConstants.ampPosition));
    speakerPosition.onTrue(new SetArmPositionCMD(arm, Constants.OperatorConstants.speakerPosition));
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
    double waitTime = SmartDashboard.getNumber("Wait Time", 0);
    boolean autoDrive = SmartDashboard.getBoolean("Auto Drive", false);
    boolean autoShoot = SmartDashboard.getBoolean("Auto Shoot", false);

    if(autoDrive && autoShoot) return new ScuffedAutoCommand(s_Swerve, launcher, feed, intake, arm, waitTime);

    return new SequentialCommandGroup(
      new WaitCommand(waitTime),
      autoShoot? new SpeakerLaunchCMD(launcher, feed, intake, arm) : new WaitCommand(0.1),
      autoDrive? new DriveStraightCMD(s_Swerve) : new WaitCommand(0.1)
    );
    // return autoChooser.getSelected();
  }
}


// // Copyright (c) FIRST and other WPILib contributors.
// // Open Source Software; you can modify and/or share it under the terms of
// // the WPILib BSD license file in the root directory of this project.

// package frc.robot;

// import com.pathplanner.lib.auto.AutoBuilder;
// import com.pathplanner.lib.auto.NamedCommands;

// import edu.wpi.first.wpilibj.GenericHID;
// import edu.wpi.first.wpilibj.Joystick;
// import edu.wpi.first.wpilibj.XboxController;
// import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
// import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
// import edu.wpi.first.wpilibj2.command.Command;
// import edu.wpi.first.wpilibj2.command.WaitCommand;
// import edu.wpi.first.wpilibj2.command.InstantCommand;
// import edu.wpi.first.wpilibj2.command.button.JoystickButton;
// import frc.robot.commands.manual.ArmJoystickCMD;
// import frc.robot.commands.manual.FeedTriggerCMD;
// import frc.robot.commands.manual.IntakeTriggerCMD;
// import frc.robot.commands.manual.LauncherTriggerCMD;
// import frc.robot.commands.manual.TeleopSwerve;
// import frc.robot.commands.presets.*;
// import frc.robot.subsystems.*;

// /**
//  * This class is where the bulk of the robot should be declared. Since Command-based is a
//  * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
//  * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
//  * subsystems, commands, and button mappings) should be declared here.
//  */
// public class RobotContainer {
//     /* Subsystems */
//   public final Swerve s_Swerve = new Swerve();
//   public final Arm arm = new Arm();
//   public final Intake intake = new Intake();
//   public final Launcher launcher = new Launcher();
//   public final Feed feed = new Feed();
//   public final Climber climber = new Climber();
//   public final BeamBreak beamBreak = new BeamBreak();

//   /* Controllers */
//   private final Joystick driver = new Joystick(0);
//   private final Joystick operator = new Joystick(1);

// //driver
// //lt intake
// //rt shoot
// //lb slow mode
// //rb outtake
// //x is lock
// //y is speaker
// //b is amp
// //a is intake arm position

// //operator
// //arm as is, feed/intake as is
// //rt launch reverse
// //lt launch normal
// //y zero gyro
// //a robotcentrix
// //b climb

//   /* Driver Buttons */
//   private final JoystickButton zeroGyro = new JoystickButton(operator, Constants.ControllerMap.y);
//   private final JoystickButton robotCentric = new JoystickButton(operator, Constants.ControllerMap.a);
//   private final JoystickButton climb = new JoystickButton(operator, Constants.ControllerMap.b);

//   private final JoystickButton slow = new JoystickButton(driver, Constants.ControllerMap.LB);
//   private final JoystickButton pickup = new JoystickButton(driver, Constants.ControllerMap.leftTrigger);

//   private final JoystickButton outtake = new JoystickButton(driver, Constants.ControllerMap.RB);
  
//   private final JoystickButton pickupPosition = new JoystickButton(driver, Constants.ControllerMap.b);
//   private final JoystickButton ampPosition = new JoystickButton(driver, Constants.ControllerMap.a);
//   private final JoystickButton speakerPosition = new JoystickButton(driver, Constants.ControllerMap.y);
//   private final JoystickButton lock = new JoystickButton(driver, Constants.ControllerMap.x);

//   /** The container for the robot. Contains subsystems, OI devices, and commands. */

//   private SendableChooser<Command> autoChooser;
  
//   public RobotContainer() {
//     SmartDashboard.putBoolean("seneca auto", false);
    
//     // s_Swerve.setDefaultCommand(
//     //     new NaiveAutoAlignCMD(
//     //         s_Swerve,
//     //         () -> -driver.getRawAxis(translationAxis),
//     //         () -> driver.getRawAxis(strafeAxis),
//     //         () -> robotCentric.getAsBoolean(),
//     //         new int[]{5}));

//     s_Swerve.setDefaultCommand(
//         new TeleopSwerve(
//             s_Swerve,
//             () -> driver.getRawAxis(Constants.ControllerMap.leftStickY),
//             () -> driver.getRawAxis(Constants.ControllerMap.leftStickX),
//             () -> -driver.getRawAxis(Constants.ControllerMap.rightStickX),
//             () -> robotCentric.getAsBoolean()));

//     arm.setDefaultCommand(
//       new ArmJoystickCMD(
//         arm,
//         () -> -operator.getRawAxis(Constants.ControllerMap.leftStickY)
//       )
//     );

//     launcher.setDefaultCommand(
//       new LauncherTriggerCMD(
//         launcher,
//         () -> operator.getRawAxis(Constants.ControllerMap.leftTrigger),
//         () -> operator.getRawAxis(Constants.ControllerMap.rightTrigger)
//       )
//     );

//     feed.setDefaultCommand(
//         new FeedTriggerCMD(
//         feed,
//         () -> operator.getRawAxis(Constants.ControllerMap.leftTrigger),
//         () -> operator.getRawAxis(Constants.ControllerMap.rightStickY)
//       )
//     );

//     intake.setDefaultCommand(
//       new IntakeTriggerCMD(
//         intake,
//         () -> operator.getRawAxis(Constants.ControllerMap.rightStickY)
//       )
//     );
    
//     //Register Named Commands - Temporary
//     NamedCommands.registerCommand("Lock", new LockCMD(s_Swerve));
//     NamedCommands.registerCommand("Shoot", new SpeakerLaunchCMD(launcher, feed, intake, arm));
//     NamedCommands.registerCommand("Pickup", new IntakeCMD(intake, arm, feed));
//     // Configure the button bindingszeroGyro
//     configureButtonBindings();
    

//     autoChooser = AutoBuilder.buildAutoChooser();
//     SmartDashboard.putData("Select Auto", autoChooser);

//     SmartDashboard.putNumber("Wait Time", 0);
//   }

//   /**
//    * Use this method to define your button->command mappings. Buttons can be created by
//    * instantiating a {@link GenericHID} or one of its subclasses ({@link
//    * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
//    * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
//    */
//   private void configureButtonBindings() {
//     /* Driver Buttons */
//     zeroGyro.onTrue(new InstantCommand(() -> s_Swerve.zeroGyro()));
//     lock.onTrue(new LockCMD(s_Swerve));
//     slow.onTrue(new SetSwerveSpeedCMD(s_Swerve, 0.3)).onFalse(new SetSwerveSpeedCMD(s_Swerve,1));
//     ampPosition.onTrue(new SetArmPositionCMD(arm, Constants.OperatorConstants.ampPosition));
//     // speakerPosition.onTrue(new SetArmPositionCMD(arm, Constants.OperatorConstants.speakerPosition));
//     // pickupPosition.onTrue(new SetArmPositionCMD(arm, Constants.OperatorConstants.intakePosition));
//     ampPosition.onTrue(new AmpLaunchCMD(launcher, feed, intake, arm));
//     speakerPosition.onTrue(new SpeakerLaunchCMD(launcher, feed, intake, arm));
//     pickupPosition.onTrue(new SetArmPositionCMD(arm, Constants.OperatorConstants.intakePosition));

//     climb.onTrue(new InstantCommand(()-> climber.toggle()));
//     pickup.onTrue(new IntakeCMD(intake, arm, feed));
//     outtake.onTrue(new OuttakeCMD(intake, feed));
//     // launchWithoutArm.onTrue(new S(launcher, feed, intake));
//   }
  
//   public void teleopInit(){
//     s_Swerve.resetEncoders();
//     //REMEMBER THIS LINE
//     s_Swerve.zeroGyro();
//   }

//   /**
//    * Use this to pass the autonomous command to the main {@link Robot} class.
//    *
//    * @return the command to run in autonomous
//    */
//   public Command getAutonomousCommand() {
//     // An ExampleCommand will run in autonomous
//     //return new exampleAuto(s_Swerve);
//     if(SmartDashboard.getBoolean("seneca auto", false)){
//       return new DriveStraightCMD(s_Swerve);
//     }else{
//       return new WaitCommand(1);
//     }
//     // return autoChooser.getSelected();
//   }
// }
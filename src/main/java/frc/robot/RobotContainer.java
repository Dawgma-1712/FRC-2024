// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.HashMap;

import com.pathplanner.lib.auto.*;
import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.XboxController.Button;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.*;
import frc.robot.subsystems.*;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.Commands;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  
  /* Controllers */
  private final Joystick driver = new Joystick(OperatorConstants.DriverControllerPort);
  private final Joystick operator = new Joystick(OperatorConstants.OperatorControllerPort);

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
  private final JoystickButton slow = new JoystickButton(driver, 6);
  private final JoystickButton autoAlign = new JoystickButton(driver, 1);
  private final JoystickButton climb = new JoystickButton(driver, 4);
  private final JoystickButton intakeB = new JoystickButton(driver, Button.kLeftBumper.value);
  private final JoystickButton launch = new JoystickButton(driver, Button.kRightBumper.value);

  /* Subsystems */
  private final Swerve s_Swerve = new Swerve();
  private final Arm arm = new Arm();
  private final Extender extender = new Extender();
  private final Launcher launcher = new Launcher();
  private final Intake intake = new Intake();
  private final BeamBreak beamBreak = new BeamBreak();
  private LED color = new LED();
  private final Vision limelight = new Vision();



  /** The container for the robot. Contains subsystems, OI devices, and commands. */

  private SendableChooser<Command> autoChooser;

  public RobotContainer() {
    s_Swerve.setDefaultCommand(
        new TeleopSwerve(
            s_Swerve,
            () -> -driver.getRawAxis(translationAxis),
            () -> driver.getRawAxis(strafeAxis),
            () -> -driver.getRawAxis(rotationAxis),
            () -> robotCentric.getAsBoolean()));
    
    arm.setDefaultCommand(new ArmJoystickCommand(
      arm,
      () -> -operator.getRawAxis(OperatorConstants.OperatorArm)
     )); 

    // Configure the button bindings
    configureButtonBindings();
    

    autoChooser = AutoBuilder.buildAutoChooser();
    SmartDashboard.putData("Select Auto", autoChooser);

    //Register Named Commands
    NamedCommands.registerCommand("Shoot", Commands.print("PLACEHOLDER"));
    NamedCommands.registerCommand("Pickup", Commands.print("PLACEHOLDER"));
    NamedCommands.registerCommand("Lock", new Lock(s_Swerve));
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
    /* Driver Buttons */
    //zeroGyro.whenPressed(new InstantCommand(() -> s_Swerve.zeroGyro()));
    lock.onTrue(new Lock(s_Swerve));
    slow.onTrue(new SlowMode(s_Swerve, 0.5)).onFalse(new SlowMode(s_Swerve, 3));
    autoAlign.onTrue(new AutoAlign(s_Swerve, limelight));
    climb.onTrue(new Climber(arm, extender));
    launch.onTrue(new LauncherCMD(launcher, beamBreak));
    intakeB.onTrue(new IntakeCMD(intake, beamBreak));

    new Trigger(beamBreak::beamBreak)
        .onTrue(new AutoAlign(s_Swerve, limelight));

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
  public int[] returnColor(){
    return color.currentRGB();
  }
}
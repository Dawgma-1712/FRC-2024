package frc.robot.commands;

import frc.robot.Constants;
import frc.robot.subsystems.*;
import frc.robot.commands.*;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;

import java.util.function.Supplier;

public class LaunchCMD extends SequentialCommandGroup{  
    Launcher launcher;
    Feed feed;
    Intake intake;
    Arm arm;

    public LaunchCMD(Launcher launcher, Feed feed, Intake intake, Arm arm){
        this.launcher = launcher;
        this.arm = arm;
        this.feed = feed;
        this.intake = intake;
        addRequirements(launcher, arm, feed, intake);
        addCommands(
            
            new SetLauncherSpeedCMD(launcher, Constants.EndEffectorConstants.launchSpeed),
            new InstantCommand(() -> intake.setSpeed(Constants.EndEffectorConstants.intakeSpeed)),
            new InstantCommand(() -> feed.setSpeed(Constants.EndEffectorConstants.feedSpeed)),
            new WaitCommand(Constants.EndEffectorConstants.launchTime)
        );
    }
}

    
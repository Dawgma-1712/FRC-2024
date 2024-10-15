package frc.robot.commands.presets;

import frc.robot.Constants;
import frc.robot.subsystems.*;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;

public class SpeakerLaunchCMD extends SequentialCommandGroup{  
    private double speed = 0;
    private Rotation2d angle =Rotation2d.fromDegrees(30);
    public SpeakerLaunchCMD(Launcher launcher, Feed feed, Intake intake, Arm arm){
        speed = Constants.EndEffectorConstants.speakerLaunchSpeed;
        angle = Constants.OperatorConstants.speakerPosition;
        
        addRequirements(launcher, arm, feed, intake);
        //decent chance that the speeds will go back to defualt too fast
        addCommands(
            // new InstantCommand(() -> arm.setTargetPosition(angle)), 
            new SetArmPositionCMD(arm, angle),
            new WaitCommand(0.5),
            new InstantCommand(() -> feed.setSpeed(-Constants.EndEffectorConstants.feedSpeed)),
            new InstantCommand(() -> intake.setSpeed(-Constants.EndEffectorConstants.intakeSpeed)),
            new WaitCommand(0.1),
            new InstantCommand(() -> feed.setSpeed(0)),
            new InstantCommand(() -> intake.setSpeed(0)),
            new InstantCommand(() -> launcher.setSpeed(speed)),
            new WaitCommand(Constants.EndEffectorConstants.launchWaitTime),
            new InstantCommand(() -> intake.setSpeed(Constants.EndEffectorConstants.intakeSpeed)),
            new InstantCommand(() -> feed.setSpeed(Constants.EndEffectorConstants.launchFeedSpeed)),
            new WaitCommand(Constants.EndEffectorConstants.launchContactTime),
            new InstantCommand(() -> launcher.setSpeed(0)),
            new InstantCommand(() -> intake.setSpeed(0)),
            new InstantCommand(() -> feed.setSpeed(0))
        );
    }
}

    
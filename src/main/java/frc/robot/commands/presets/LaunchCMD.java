package frc.robot.commands.presets;

import frc.robot.Constants;
import frc.robot.subsystems.*;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;

public class LaunchCMD extends SequentialCommandGroup{  
    private double speed = 0;
    private Rotation2d angle =Rotation2d.fromDegrees(30);
    public LaunchCMD(Launcher launcher, Feed feed, Intake intake, Arm arm, String target){
        if(target == "amp"){
            speed = Constants.EndEffectorConstants.ampLaunchSpeed;
            angle = Constants.OperatorConstants.ampPosition;
        }else if(target == "speaker"){
            speed = Constants.EndEffectorConstants.speakerLaunchSpeed;
            angle = Constants.OperatorConstants.speakerPosition;
        }
        
        addRequirements(launcher, arm, feed, intake);
        //decent chance that the speeds will go back to defualt too fast
        addCommands(
            // new SetLauncherSpeedCMD(launcher, Constants.EndEffectorConstants.launchSpeed),
            new InstantCommand(() -> arm.setTargetPosition(angle)), 
            new InstantCommand(() -> launcher.setSpeed(1)),
            new WaitCommand(Constants.EndEffectorConstants.launchWaitTime),
            // new InstantCommand(() -> feed.setSpeed(Constants.EndEffectorConstants.launchFeedSpeed)),
            // new WaitCommand(0.1),
            new InstantCommand(() -> intake.setSpeed(Constants.EndEffectorConstants.intakeSpeed)),
            new InstantCommand(() -> feed.setSpeed(1)),
            new WaitCommand(Constants.EndEffectorConstants.launchContactTime),
            new InstantCommand(() -> launcher.setSpeed(0)),
            new InstantCommand(() -> intake.setSpeed(0)),
            new InstantCommand(() -> feed.setSpeed(0))
        );
    }
}

    
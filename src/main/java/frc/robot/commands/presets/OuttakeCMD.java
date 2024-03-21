package frc.robot.commands.presets;

import frc.robot.Constants;
import frc.robot.subsystems.*;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;


public class OuttakeCMD extends SequentialCommandGroup{
    Intake intake;
    Feed feed;

    public OuttakeCMD(Intake intake, Feed feed){
        this.intake = intake;
        this.feed = feed;
        addRequirements(intake, feed);
        
        addCommands(
            new InstantCommand(() -> intake.setSpeed(-Constants.EndEffectorConstants.intakeSpeed)),
            new InstantCommand(() -> feed.setSpeed(-Constants.EndEffectorConstants.feedSpeed)),
            new WaitCommand(Constants.EndEffectorConstants.intakeTime),
            new InstantCommand(() -> intake.setSpeed(0)),
            new InstantCommand(() -> feed.setSpeed(0))
        );
    }
}

    
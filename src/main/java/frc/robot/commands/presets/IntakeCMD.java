package frc.robot.commands.presets;

import frc.robot.Constants;
import frc.robot.subsystems.*;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;


public class IntakeCMD extends SequentialCommandGroup{
    Intake intake;
    Arm arm;
    Feed feed;

    public IntakeCMD(Intake intake, Arm arm, Feed feed){
        this.arm = arm;
        this.intake = intake;
        this.feed = feed;
        addRequirements(arm, intake, feed);
        
        addCommands(
            new SetArmPositionCMD(arm, Constants.OperatorConstants.intakePosition),
            new InstantCommand(() -> intake.setSpeed(Constants.EndEffectorConstants.intakeSpeed)),
            new InstantCommand(() -> feed.setSpeed(Constants.EndEffectorConstants.feedSpeed)),
            new WaitCommand(Constants.EndEffectorConstants.intakeTime),
            new InstantCommand(() -> intake.setSpeed(0)),
            new InstantCommand(() -> feed.setSpeed(0))
        );
    }
}

    
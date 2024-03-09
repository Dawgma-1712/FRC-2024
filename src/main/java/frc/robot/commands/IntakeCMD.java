package frc.robot.commands;

import frc.robot.Constants;
import frc.robot.subsystems.*;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;


public class IntakeCMD extends SequentialCommandGroup{
    Intake intake;
    Arm arm;

    public IntakeCMD(Intake intake, Arm arm){
        this.arm = arm;
        this.intake = intake;
        addRequirements(arm, intake);
        
        addCommands(
            new SetArmPositionCMD(arm, Constants.OperatorConstants.nearLaunchPosition),
            new InstantCommand(() -> intake.setSpeed(Constants.EndEffectorConstants.intakeSpeed))
        );
    }
}

    
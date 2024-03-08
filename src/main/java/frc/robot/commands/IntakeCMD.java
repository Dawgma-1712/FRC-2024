package frc.robot.commands;

import frc.robot.Constants;
import frc.robot.subsystems.*;
import frc.robot.commands.*;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;

import java.util.function.Supplier;

public class IntakeCMD extends SequentialCommandGroup{
    Intake intake;
    Arm arm;

    public IntakeCMD(Intake intake, Arm arm){
        this.arm = arm;
        this.intake = intake;
        addRequirements(arm, intake);
        addCommands(
            new InstantCommand(() -> intake.setSpeed(Constants.EndEffectorConstants.intakeSpeed)),
            new InstantCommand(() -> arm.setTargetPosition(Constants.OperatorConstants.intakePos))
        );
    }
}

    
package frc.robot.commands;

import frc.robot.Constants;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.BeamBreak;
import frc.robot.subsystems.Intake;
import edu.wpi.first.wpilibj2.command.Command;
import java.util.function.Supplier;

public class IntakeCMD extends Command{   
    Intake intake;
    Arm arm;

    public IntakeCMD(Intake intake, Arm arm){
        this.intake = intake;
        this.arm = arm;
        addRequirements(intake, arm);
    }

    @Override
    public void initialize() {
     arm.setGoalState(Constants.OperatorConstants.intakePos);
    }


    @Override
    public void execute(){
        intake.intake();
    }

    @Override
    public void end(boolean interrupted){
        intake.stop();
    }

    @Override
    public boolean isFinished(){
        return false;
    }


}

    
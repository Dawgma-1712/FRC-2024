package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.OperatorConstants;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.BeamBreak;

public class ArmPIDCommand extends Command{
    private final Arm arm;

    public ArmPIDCommand(Arm arm){
        this.arm = arm;
        addRequirements(arm);
    }

    public void initialize(){

    }

    public void execute(){
    }
    
    public void end(boolean interrupted){
        arm.setIdle();
    }
    public boolean isFinished(){
        return false;
    }
}
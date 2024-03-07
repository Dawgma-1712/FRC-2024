package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.LimelightHelpers;
import frc.robot.Constants.OperatorConstants;
import frc.robot.subsystems.Arm;

public class ArmLimeLightCommand extends Command{
    
    private final Arm arm;

    public ArmLimeLightCommand(Arm arm){
        this.arm = arm;
        addRequirements(arm);
    }

    public void initialize(){
        if(LimelightHelpers.getFiducialID("") == 4 || LimelightHelpers.getFiducialID("") == 7) {
            if(LimelightHelpers.getBotPose3d_TargetSpace("").getX() < Constants.OperatorConstants.launchMidpoint) {
                arm.setGoalState(Constants.OperatorConstants.launchPos1);
            }

            else {
                arm.setGoalState(Constants.OperatorConstants.launchPos2);
            }
        }

        else {
            arm.setGoalState(Constants.OperatorConstants.ampPos);
        }
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
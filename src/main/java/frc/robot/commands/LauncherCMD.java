package frc.robot.commands;

import frc.robot.LimelightHelpers;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.BeamBreak;
import frc.robot.subsystems.Launcher;
import edu.wpi.first.wpilibj2.command.Command;
import java.util.function.Supplier;

public class LauncherCMD extends Command{
    Launcher launcher;
    Arm arm;

    public LauncherCMD(Launcher launcher, Arm arm){
        this.launcher = launcher;
        this.arm = arm;
        addRequirements(launcher, arm);
    }

    @Override
    public void initialize() {
    }


    @Override
    public void execute(){
        if(LimelightHelpers.getTV("")) {
        arm.setGoalState((int)(arm.speakerAngle()));
        }
        
        if(Math.abs(arm.getRaise1Position() - arm.speakerAngle()) < 5 && LimelightHelpers.getBotPose3d_TargetSpace("").getX() < 3) {
            launcher.launch();
        }
    }

    @Override
    public void end(boolean interrupted){
        launcher.stop();
    }

    @Override
    public boolean isFinished(){
        return false;
    }


}

    
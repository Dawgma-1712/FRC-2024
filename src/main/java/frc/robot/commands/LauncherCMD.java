package frc.robot.commands;

import frc.robot.subsystems.Arm;
import frc.robot.subsystems.BeamBreak;
import frc.robot.subsystems.Launcher;
import edu.wpi.first.wpilibj2.command.Command;
import java.util.function.Supplier;

public class LauncherCMD extends Command{  
    Launcher launcher;

    public LauncherCMD(Launcher launcher, Arm arm){
        this.launcher = launcher;
        addRequirements(launcher, arm);
    }

    @Override
    public void initialize() {
     
    }


    @Override
    public void execute(){
        launcher.outtake();
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

    
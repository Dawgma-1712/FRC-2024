package frc.robot.commands;

import frc.robot.subsystems.BeamBreak;
import frc.robot.subsystems.Launcher;
import edu.wpi.first.wpilibj2.command.Command;
import java.util.function.Supplier;

public class LauncherCMD extends Command{
    private boolean previous;    
    Launcher launcher;
    BeamBreak sensor;

    public LauncherCMD(Launcher launcher, BeamBreak sensor){
        this.launcher = launcher;
        this.sensor = sensor;
        addRequirements(launcher);
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
        return !sensor.beamBreak();
    }


}

    
package frc.robot.commands;


import edu.wpi.first.wpilibj2.command.*;
import frc.robot.subsystems.*;

public class LockCMD extends Command{
    private final Swerve swerve;

    public LockCMD(Swerve swerve){
        this.swerve = swerve;
        addRequirements(swerve);
    }

    @Override
    public void initialize(){

    }

    @Override
    public void execute(){
        swerve.toggleLock();
    }

    @Override
    public void end(boolean interrupted){
        swerve.stopModules();
    }

    @Override
    public boolean isFinished(){
        return true;
    }
}

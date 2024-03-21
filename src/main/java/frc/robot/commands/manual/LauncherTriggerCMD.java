package frc.robot.commands.manual;
import java.util.function.Supplier;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.*;

public class LauncherTriggerCMD extends Command{
    private Launcher launcher;
    private Supplier<Double> speedForwards;
    private Supplier<Double> speedBackwards;

    public LauncherTriggerCMD(Launcher launcher, Supplier<Double> speedForwards, Supplier<Double> speedBackwards) {
        this.launcher = launcher;
        this.speedForwards = speedForwards;
        this.speedBackwards = speedBackwards;
        addRequirements(launcher);
    }

    public void initialize() {}

    @Override
    public void execute() {
        double speed;
        if(speedForwards.get() > speedBackwards.get() ){
            speed = speedForwards.get();
        }else{
            speed = -speedBackwards.get();
        }

        if(Math.abs(speed) > Constants.OperatorConstants.LauncherDeadband){
            launcher.setSpeed(speed);
        }else{
            launcher.setSpeed(0);
        }
    }

    @Override
    public void end(boolean interrupted) {
        launcher.stop();
    }

    @Override
    public boolean isFinished() {
    return false;
    }
}
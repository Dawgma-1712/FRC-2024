package frc.robot.commands;
import java.util.function.Supplier;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.*;

public class LauncherTriggerCMD extends Command{
    private Launcher launcher;
    private Supplier<Double> speed;

    public LauncherTriggerCMD(Launcher launcher, Supplier<Double> speed) {
        this.launcher = launcher;
        this.speed = speed;
        addRequirements(launcher);
    }

    public void initialize() {}

    @Override
    public void execute() {
        if(speed.get() < Constants.OperatorConstants.LauncherDeadband) launcher.setSpeed(speed.get());
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
package frc.robot.commands;
import java.util.function.Supplier;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.*;

public class FeedTriggerCMD extends Command{
    private Feed feed;
    private Supplier<Double> intakeSpeed, launcherSpeed;

    public FeedTriggerCMD(Feed feed, Supplier<Double> intakeSpeed, Supplier<Double> launcherSpeed) {
        this.feed = feed;
        this.intakeSpeed = intakeSpeed;
        this.launcherSpeed = launcherSpeed;
        addRequirements(feed);
    }

    public void initialize() {}

    @Override
    public void execute() {
      if(Math.abs(intakeSpeed.get()) > Math.abs(launcherSpeed.get())){
        feed.setSpeed(intakeSpeed.get());
      }
      else{
        feed.setSpeed(launcherSpeed.get());
      }
    }

    @Override
    public void end(boolean interrupted) {
        feed.stop();
    }

    @Override
    public boolean isFinished() {
    return false;
    }
}
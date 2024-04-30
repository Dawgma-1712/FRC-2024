package frc.robot.commands.manual;
import java.util.function.Supplier;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.*;
import frc.robot.Constants;

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
      double speed = 0;
      if(Math.abs(intakeSpeed.get()) > Math.abs(launcherSpeed.get())){
        speed = intakeSpeed.get();
       
      }
      else{
        speed = launcherSpeed.get();
      }
      if(Math.abs(speed) > Constants.EndEffectorConstants.feedDeadband){
        feed.setSpeed(speed);
      }else{
        feed.setSpeed(0);
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
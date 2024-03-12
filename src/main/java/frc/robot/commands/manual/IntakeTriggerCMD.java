package frc.robot.commands.manual;
import java.util.function.Supplier;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.*;

public class IntakeTriggerCMD extends Command{
    private Intake intake;
    private Supplier<Double> speed;

    public IntakeTriggerCMD(Intake intake, Supplier<Double> speed) {
        this.intake = intake;
        this.speed = speed;
        addRequirements(intake);
    }

    public void initialize() {}

    @Override
    public void execute() {
        intake.setSpeed(speed.get());
    }

    @Override
    public void end(boolean interrupted) {
        intake.stop();
    }

    @Override
    public boolean isFinished() {
    return false;
    }
}
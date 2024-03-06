package frc.robot.commands;
import java.util.function.Supplier;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.*;

public class IntakeTriggerCMD extends CommandBase{
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
        intake.setSpeed(speed.get() / 2);
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
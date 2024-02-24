package frc.robot.commands;
import java.util.function.Supplier;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.*;

public class ArmJoystickCommand extends CommandBase{
    private Arm arm;
    private Supplier<Double> raiseValue;

    public ArmJoystickCommand(Arm arm, Supplier<Double> raiseValue) {
        this.arm = arm;
        this.raiseValue = raiseValue;
        addRequirements(arm);
    }

    public void initialize() {}

    @Override
    public void execute() {
        double raise = Math.abs(raiseValue.get()) > 0.04 ? raiseValue.get() : 0;
        arm.manualArm(raise);
    }

    @Override
    public void end(boolean interrupted) {

    }

    @Override
    public boolean isFinished() {
    return false;
    }
}
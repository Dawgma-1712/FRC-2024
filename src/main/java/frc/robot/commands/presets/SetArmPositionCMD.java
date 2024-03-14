package frc.robot.commands.presets;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.*;
import frc.robot.Constants;


public class SetArmPositionCMD extends Command{
    private Arm arm;
    private double position;

    public SetArmPositionCMD(Arm arm, double position) {
        this.arm = arm;
        this.position = position;
        addRequirements(arm);
    }

    public void initialize() {}

    @Override
    public void execute() {
        arm.setTargetPosition(position);
    }

    @Override
    public void end(boolean interrupted) {
    }

    @Override
    public boolean isFinished() {
        return Math.abs(arm.getPosition()-position) < Constants.EndEffectorConstants.armThreshold;
    }
}
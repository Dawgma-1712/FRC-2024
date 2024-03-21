package frc.robot.commands.presets;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.*;
import frc.robot.Constants;


public class SetArmPositionCMD extends Command{
    private Arm arm;
    private Rotation2d angle;

    public SetArmPositionCMD(Arm arm, Rotation2d angle) {
        this.arm = arm;
        this.angle = angle;
        addRequirements(arm);
    }

    public void initialize() {}

    @Override
    public void execute() {
        arm.setTargetPosition(angle);
    }

    @Override
    public void end(boolean interrupted) {
    }

    @Override
    public boolean isFinished() {
        return true;
        // return Math.abs(arm.getAngle().getDegrees()-angle.getDegrees()) < Constants.EndEffectorConstants.armThreshold;
    }
}
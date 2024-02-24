package frc.robot.commands;

import java.util.function.Supplier;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Swerve;
import frc.robot.subsystems.Vision;
import frc.robot.Constants;

public class AutoAlign extends Command{

    private Swerve swerve;
    private Vision vision;
    private Supplier<Double> tx;
    private Supplier<Double> ty;
    private Supplier<Double> ta;
    private Supplier<Double> tid;
    private final double kP = 1.0/104;
    private Double[] ids = Constants.OperatorConstants.id_Blue;

    public AutoAlign(Swerve swerve, Vision vision, Supplier<Double> tx, Supplier<Double> ty, Supplier<Double> ta, Supplier<Double> tid) {
        this.swerve = swerve;
        this.vision = vision;
        this.tx = tx;
        this.ty = ty;
        this.ta = ta;
        this.tid = tid;

        addRequirements(swerve);
    }

    public void initialize() {}

    public void execute() {
    }

    public void end(boolean interrupted) {}

    public boolean isFinished() {
        return true;
    }
    
}
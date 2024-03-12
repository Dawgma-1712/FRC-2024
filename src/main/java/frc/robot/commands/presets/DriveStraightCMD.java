package frc.robot.commands.presets;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.Swerve;
import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

public class DriveStraightCMD extends Command {
    private Swerve s_Swerve;
    private double start_time;

    public DriveStraightCMD(Swerve s_Swerve) {
        this.s_Swerve = s_Swerve;
        this.start_time = Timer.getFPGATimestamp();
        addRequirements(s_Swerve);
    }

    @Override
    public void execute() {
        /* Get Values, Deadband*/
        s_Swerve.drive(
            new Translation2d(-0.5, 0).times(Constants.Swerve.maxSpeed),
            0,
            true,
            false);
    }
  
    @Override
    public void end(boolean interrupted) {
        s_Swerve.stopModules();
    }

    @Override
    public boolean isFinished() {
        // TODO Auto-generated method stub
        return Timer.getFPGATimestamp()-this.start_time > 3;
    }
  
}
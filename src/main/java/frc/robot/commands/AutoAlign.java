package frc.robot.commands;

import java.util.function.Supplier;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Swerve;
import frc.robot.subsystems.Vision;
import frc.robot.Constants;

public class AutoAlign extends Command{

    private Swerve swerve;
    private Vision vision;
    private double xGoal, yGoal, yawGoal;
    private double driveVel, strafeVel, rotateVel;
    private PIDController drivePID, strafePID, rotationPID;
    private final double kP = 1.0/104;
    private Double[] ids = Constants.OperatorConstants.id_Blue;

    final SlewRateLimiter translationLimiter = new SlewRateLimiter(3.0);
    final SlewRateLimiter strafeLimiter = new SlewRateLimiter(3.0);
    final SlewRateLimiter rotationLimiter = new SlewRateLimiter(3.0);

    public AutoAlign(Swerve swerve, Vision vision) {
        this.swerve = swerve;
        this.vision = vision;

    final SlewRateLimiter translationLimiter = new SlewRateLimiter(3.0);
    final SlewRateLimiter strafeLimiter = new SlewRateLimiter(3.0);
    final SlewRateLimiter rotationLimiter = new SlewRateLimiter(3.0);

    this.drivePID = new PIDController(
            Constants.Swerve.driveKP, 
            Constants.Swerve.driveKI, 
            Constants.Swerve.driveKD);

        this.strafePID = new PIDController(
            Constants.Swerve.driveKP, 
            Constants.Swerve.driveKI, 
            Constants.Swerve.driveKD);

        this.rotationPID = new PIDController(
            Constants.Swerve.angleKP, 
            Constants.Swerve.angleKI, 
            Constants.Swerve.angleKD);


        xGoal = Constants.OperatorConstants.aprilTagX[(int)(vision.getTID()) - 1];
        yGoal = Constants.OperatorConstants.aprilTagY[(int)(vision.getTID()) - 1];
        yawGoal = Constants.OperatorConstants.aprilTagYaw[(int)(vision.getTID()) - 1];

        addRequirements(swerve);
    }

    public void initialize() {

    }

    public void execute() {
        driveVel = drivePID.calculate(vision.getFieldX(), xGoal);
        strafeVel = drivePID.calculate(vision.getFieldY(), yGoal);
        rotateVel = drivePID.calculate(vision.getFieldYaw(), yawGoal);

        driveVel = driveVel > Constants.OperatorConstants.deadband ? driveVel : 0;
        strafeVel = strafeVel > Constants.OperatorConstants.deadband ? strafeVel : 0;
        rotateVel = rotateVel > Constants.OperatorConstants.deadband ? rotateVel : 0;

        driveVel = translationLimiter.calculate(driveVel) * 3;
        strafeVel = strafeLimiter.calculate(strafeVel) * 3;
        rotateVel = rotationLimiter.calculate(rotateVel) * 5;

        ChassisSpeeds chassisSpeeds = new ChassisSpeeds(driveVel, strafeVel, rotateVel);

        SwerveModuleState[] swerveModuleStates = Constants.Swerve.swerveKinematics.toSwerveModuleStates(chassisSpeeds);

        swerve.setModuleStates(swerveModuleStates);
    }

    public void end(boolean interrupted) {
        swerve.stopModules();
    }

    public boolean isFinished() {
        return Math.abs(vision.getFieldX() - xGoal) < 10
        && Math.abs(vision.getFieldY() - yGoal) < 10
        && Math.abs(vision.getFieldYaw() - yawGoal) < 10;
    }
    
}